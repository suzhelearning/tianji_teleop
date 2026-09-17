// Original AdaptiveOptimizerAnalytical equations, executed without Python callbacks.
#include <pinocchio/parsers/urdf.hpp>
#include <pinocchio/algorithm/kinematics.hpp>
#include <pinocchio/algorithm/jacobian.hpp>
#include <pinocchio/algorithm/frames.hpp>
#include <nlopt.h>
#include <array>
#include <algorithm>
#include <cmath>
#include <memory>
#include <stdexcept>
#include <string>
#include <thread>

namespace {
using Vec = Eigen::Matrix<double,20,1>;
using V3 = Eigen::Vector3d;
using Jac = Eigen::Matrix<double,3,20>;
using Points = Eigen::Matrix<double,21,3,Eigen::RowMajor>;
thread_local std::string error;
void require(bool ok, const char* message) { if (!ok) throw std::invalid_argument(message); }
void finite(const double* p, std::size_t n) {
    require(p, "null numeric input");
    for (std::size_t i=0;i<n;++i) require(std::isfinite(p[i]), "nonfinite numeric input");
}
double huber(double d, double delta) { return d<=delta ? .5*d*d : delta*(d-.5*delta); }
double huber_grad(double d, double delta) { return d<=delta ? d : delta; }

struct Targets {
    std::array<V3,5> tip, dir;
    std::array<V3,15> full;
    std::array<double,5> alpha{};
};

class Optimizer {
public:
    pinocchio::Model model;
    std::unique_ptr<pinocchio::Data> data;
    std::array<pinocchio::FrameIndex,16> frames{};
    std::array<int,5> pip{}, dip{};
    // delta, delta_dir, norm_delta, w_pos, w_dir, scaling, w_full,
    // thumb_skip_pip, w_hyper, soft_min, w_couple, couple_ratio,
    // segment_scaling[5][3], d1[4], d2[4].
    std::array<double,35> c{};
    Vec last=Vec::Zero(), reg=Vec::Zero();
    bool has_last=false, has_reg=false, callback_failed=false;
    Targets target;
    nlopt_opt opt=nullptr;
    std::thread::id owner=std::this_thread::get_id();

    Optimizer(const char* urdf, const char* const* names, const double* config) {
        require(urdf && *urdf && names, "missing model or names"); finite(config,35);
        std::copy(config,config+35,c.begin());
        require(c[0]>0 && c[1]>0 && (c[7]==0 || c[7]==1), "invalid loss configuration");
        pinocchio::urdf::buildModel(urdf,model);
        require(model.nq==20 && model.nv==20, "native Hand2 requires 20 scalar joints");
        std::vector<std::string> qnames;
        for (pinocchio::JointIndex i=1;i<model.joints.size();++i) {
            require(model.joints[i].nq()==1 && model.joints[i].nv()==1, "unsupported joint type");
            qnames.push_back(model.names[i]);
        }
        require(qnames.size()==20, "invalid joint count");
        for (int i=0;i<20;++i) require(names[16+i] && qnames[i]==names[16+i], "joint order mismatch");
        for (int i=0;i<16;++i) {
            require(names[i] && *names[i], "empty frame name");
            frames[i]=model.getFrameId(names[i],pinocchio::BODY);
            require(frames[i]<static_cast<pinocchio::FrameIndex>(model.nframes), "unknown frame name");
        }
        std::array<int,10> flex{};
        for (int i=0;i<5;++i) {
            pip[i]=model.joints[model.frames[frames[6+i]].parentJoint].idx_q();
            dip[i]=model.joints[model.frames[frames[11+i]].parentJoint].idx_q();
            flex[i]=pip[i]; flex[i+5]=dip[i];
        }
        std::sort(flex.begin(),flex.end());
        require(flex.front()>=0 && flex.back()<20 &&
                std::adjacent_find(flex.begin(),flex.end())==flex.end(), "invalid flex joint mapping");
        require(model.lowerPositionLimit.allFinite() && model.upperPositionLimit.allFinite() &&
                (model.lowerPositionLimit.array()<=model.upperPositionLimit.array()).all(), "invalid limits");
        data=std::make_unique<pinocchio::Data>(model);
        opt=nlopt_create(NLOPT_LD_SLSQP,20);
        if (!opt) throw std::bad_alloc();
        if (nlopt_set_maxeval(opt,50)<0 || nlopt_set_ftol_abs(opt,1e-4)<0 ||
            nlopt_set_lower_bounds(opt,model.lowerPositionLimit.data())<0 ||
            nlopt_set_upper_bounds(opt,model.upperPositionLimit.data())<0 ||
            nlopt_set_min_objective(opt,&Optimizer::callback,this)<0) {
            nlopt_destroy(opt); opt=nullptr; throw std::runtime_error("NLopt setup failed");
        }
    }
    ~Optimizer() { if(opt) nlopt_destroy(opt); }
    void check_owner() const { require(owner==std::this_thread::get_id(), "optimizer owner thread mismatch"); }

    Targets targets(const double* raw) const {
        finite(raw,63);
        const Eigen::Map<const Points> points(raw);
        Targets t;
        const V3 wrist=points.row(0).transpose();
        for (int f=0;f<5;++f) {
            const int tip=4+4*f;
            t.tip[f]=(points.row(tip).transpose()-wrist)*c[5]*100.;
            const V3 dir=(points.row(tip)-points.row(tip-1)).transpose();
            t.dir[f]=dir/(dir.norm()+1e-8);
            for (int s=0;s<3;++s)
                t.full[s*5+f]=(points.row(2+4*f+s).transpose()-wrist)*c[12+3*f+s]*100.;
            if (f>0) {
                const double distance=(points.row(tip)-points.row(4)).norm()*100.;
                t.alpha[f]=std::clamp((c[31+f-1]-distance)/(c[31+f-1]-c[27+f-1]+1e-8),0.,.7);
                t.alpha[0]=std::max(t.alpha[0],t.alpha[f]);
            }
        }
        return t;
    }

    double evaluate(const Vec& q, const Targets& t, const Vec* regularization, Vec& grad) {
        std::array<V3,16> p;
        std::array<Jac,16> j;
        // Match RobotWrapper's world-position Jacobian: R * J_LOCAL.topRows(3).
        pinocchio::forwardKinematics(model,*data,q);
        for (int i=0;i<16;++i) p[i]=pinocchio::updateFramePlacement(model,*data,frames[i]).translation()*100.;
        pinocchio::computeJointJacobians(model,*data,q);
        pinocchio::updateFramePlacements(model,*data);
        Eigen::Matrix<double,6,Eigen::Dynamic> local(6,20);
        for (int i=0;i<16;++i) {
            local.setZero();
            pinocchio::getFrameJacobian(model,*data,frames[i],pinocchio::LOCAL,local);
            j[i]=data->oMf[frames[i]].rotation()*local.topRows(3)*100.;
        }
        std::array<double,5> pos_loss{}, dir_loss{}, full_loss{};
        grad.setZero();
        for (int f=0;f<5;++f) {
            const V3 diff=p[1+f]-p[0]-t.tip[f];
            const double d=diff.norm(); pos_loss[f]=huber(d,c[0]);
            grad += (t.alpha[f]*c[3]*huber_grad(d,c[0])) *
                    ((diff/(d+1e-8)).transpose()*(j[1+f]-j[0])).transpose();
        }
        for (int f=0;f<5;++f) {
            const V3 v=p[1+f]-p[11+f]; const double n=v.norm();
            const V3 u=v/(n+1e-8), diff=u-t.dir[f]; const double d=diff.norm();
            dir_loss[f]=huber(d,c[1]);
            const Eigen::Matrix3d normal=(Eigen::Matrix3d::Identity()-u*u.transpose())/(n+1e-8);
            grad += (t.alpha[f]*c[4]*huber_grad(d,c[1])) *
                    ((diff/(d+1e-8)).transpose()*normal*(j[1+f]-j[11+f])).transpose();
        }
        for (int f=0;f<5;++f) {
            const bool skip=f==0 && c[7]!=0; const double terms=skip ? 2. : 3.;
            for (int s=0;s<3;++s) {
                if (s==0 && skip) continue;
                const int idx=(s==0 ? 6 : s==1 ? 11 : 1)+f;
                const V3 diff=p[idx]-p[0]-t.full[s*5+f]; const double d=diff.norm();
                full_loss[f]+=huber(d,c[0]);
                grad += ((1.-t.alpha[f])*c[6]/terms*huber_grad(d,c[0])) *
                        ((diff/(d+1e-8)).transpose()*(j[idx]-j[0])).transpose();
            }
            full_loss[f]/=terms;
        }
        double loss=0.;
        for (int f=0;f<5;++f)
            loss += t.alpha[f]*(c[3]*pos_loss[f]+c[4]*dir_loss[f])+(1.-t.alpha[f])*c[6]*full_loss[f];
        if (regularization) {
            const Vec diff=q-*regularization;
            loss+=c[2]*diff.squaredNorm(); grad+=2.*c[2]*diff;
        }
        for (int f=0;f<5;++f) {
            for (int idx:{pip[f],dip[f]}) {
                const double penalty=std::max(c[9]-q[idx],0.);
                loss+=c[8]*penalty*penalty; grad[idx]+=-2.*c[8]*penalty;
            }
            const double diff=q[dip[f]]-c[11]*q[pip[f]];
            loss+=c[10]*diff*diff;
            grad[dip[f]]+=2.*c[10]*diff; grad[pip[f]]+=-2.*c[10]*c[11]*diff;
        }
        require(std::isfinite(loss) && grad.allFinite(), "nonfinite loss or gradient");
        return loss;
    }

    static double callback(unsigned n, const double* x, double* gradient, void* pointer) noexcept {
        auto& self=*static_cast<Optimizer*>(pointer);
        try {
            require(n==20,"invalid NLopt dimension"); finite(x,20);
            Vec grad; const double loss=self.evaluate(Eigen::Map<const Vec>(x),self.target,
                                                     self.has_reg ? &self.reg : nullptr,grad);
            if (gradient) std::copy(grad.data(),grad.data()+20,gradient);
            return loss;
        } catch (...) {
            self.callback_failed=true; nlopt_force_stop(self.opt);
            return HUGE_VAL;
        }
    }

    int solve(const double* points, const double* explicit_q, double* output) {
        check_owner(); target=targets(points);
        if (explicit_q) {finite(explicit_q,20); reg=Eigen::Map<const Vec>(explicit_q); has_reg=true;}
        else {reg=last; has_reg=has_last;}
        Vec initial=has_reg ? reg : Vec((model.lowerPositionLimit+model.upperPositionLimit)*.5);
        initial=initial.cwiseMax(model.lowerPositionLimit).cwiseMin(model.upperPositionLimit);
        Vec q=initial; double value=0.; callback_failed=false;
        const int result=nlopt_optimize(opt,q.data(),&value);
        if (callback_failed) throw std::runtime_error("native objective failed");
        // Python catches RuntimeError (generic NLOPT_FAILURE), not RoundoffLimited/ForcedStop.
        if (result==NLOPT_FAILURE) q=initial;
        else if (result<0) throw std::runtime_error("NLopt failed with status "+std::to_string(result));
        require(q.allFinite(),"nonfinite optimizer result");
        for (int i=0;i<20;++i) q[i]=static_cast<double>(static_cast<float>(q[i]));
        require(q.allFinite(),"nonfinite float32 optimizer result");
        last=q; has_last=true; std::copy(q.data(),q.data()+20,output);
        return result==NLOPT_FAILURE ? 1 : 0;
    }
};

Optimizer& checked(void* p) {require(p,"null optimizer"); auto& s=*static_cast<Optimizer*>(p); s.check_owner(); return s;}
}

extern "C" int tianji_hand_optimizer_abi() {return 1;}
extern "C" const char* tianji_hand_optimizer_error() {return error.c_str();}
extern "C" void* tianji_hand_optimizer_create(const char* urdf, const char* const* names,
    std::size_t name_count, const double* config, std::size_t count) noexcept {
    try {require(name_count==36 && count==35,"invalid optimizer configuration size"); return new Optimizer(urdf,names,config);}
    catch (const std::exception& e) {error=e.what(); return nullptr;}
}
extern "C" void tianji_hand_optimizer_destroy(void* p) noexcept {delete static_cast<Optimizer*>(p);}
extern "C" int tianji_hand_optimizer_bind_current_thread(void* p) noexcept {
    try {require(p,"null optimizer"); static_cast<Optimizer*>(p)->owner=std::this_thread::get_id(); return 0;}
    catch (const std::exception& e) {error=e.what(); return -1;}
}
extern "C" int tianji_hand_optimizer_state(void* p, const double* input, double* output, int action) noexcept {
    try {
        auto& s=checked(p);
        require(output || (action==1 && !input),"null state output");
        if (action==1) {if(input) {finite(input,20); s.last=Eigen::Map<const Vec>(input);} s.has_last=input!=nullptr;}
        else require(action==0,"invalid state action");
        if (!s.has_last) return 1;
        require(output,"null state output"); std::copy(s.last.data(),s.last.data()+20,output); return 0;
    } catch (const std::exception& e) {error=e.what(); return -1;}
}
extern "C" int tianji_hand_optimizer_solve(void* p, const double* points, std::size_t count,
    const double* last, double* output) noexcept {
    try {require(count==63 && output,"invalid solve buffer"); return checked(p).solve(points,last,output);}
    catch (const std::exception& e) {error=e.what(); return -1;}
}
extern "C" int tianji_hand_optimizer_evaluate(void* p, const double* q, const double* points,
    std::size_t count, const double* last, double* output) noexcept {
    try {
        require(count==63 && output,"invalid evaluate buffer"); finite(q,20); if(last) finite(last,20);
        auto& s=checked(p); const Targets t=s.targets(points);
        Vec grad, reg=last ? Vec(Eigen::Map<const Vec>(last)) : Vec::Zero();
        const double cost=s.evaluate(Eigen::Map<const Vec>(q),t,last ? &reg : nullptr,grad);
        output[0]=cost; std::copy(grad.data(),grad.data()+20,output+1); return 0;
    } catch (const std::exception& e) {error=e.what(); return -1;}
}
