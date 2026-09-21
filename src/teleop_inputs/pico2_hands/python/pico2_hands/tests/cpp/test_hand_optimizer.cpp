#include <pinocchio/parsers/urdf.hpp>
#include <array>
#include <cassert>
#include <cmath>
#include <limits>
#include <string>
#include <thread>

extern "C" {
int tianji_hand_optimizer_abi();
void* tianji_hand_optimizer_create(const char*, const char* const*, std::size_t, const double*, std::size_t);
void tianji_hand_optimizer_destroy(void*);
int tianji_hand_optimizer_state(void*, const double*, double*, int);
int tianji_hand_optimizer_solve(void*, const double*, std::size_t, const double*, double*);
int tianji_hand_optimizer_evaluate(void*, const double*, const double*, std::size_t, const double*, double*);
}

int main(int argc, char** argv) {
    assert(argc==2 && tianji_hand_optimizer_abi()==1);
    assert(!tianji_hand_optimizer_create(nullptr,nullptr,0,nullptr,0));
    pinocchio::Model model; pinocchio::urdf::buildModel(argv[1],model);
    std::array<std::string,36> names;
    names[0]="r_wrist";
    const std::array<std::string,5> fingers{"thumb","index_finger","middle_finger","ring_finger","pinky"};
    for (int i=0;i<5;++i) {
        names[1+i]="r_"+fingers[i]+"_tip";
        names[6+i]="r_"+fingers[i]+"_middle";
        names[11+i]="r_"+fingers[i]+"_distal";
    }
    for (int i=0;i<20;++i) names[16+i]=model.names[i+1];
    std::array<const char*,36> ptrs;
    for (int i=0;i<36;++i) ptrs[i]=names[i].c_str();
    std::array<double,35> config{2.,.5,.04,1.,2.,1.,1.,0.,0.,0.,0.,.7};
    for (int i=12;i<27;++i) config[i]=1.;
    for (int i=27;i<31;++i) config[i]=2.;
    for (int i=31;i<35;++i) config[i]=4.;
    void* solver=tianji_hand_optimizer_create(argv[1],ptrs.data(),36,config.data(),35);
    assert(solver);
    std::array<double,20> q{}, state{};
    std::array<double,21> evaluated{};
    std::array<double,63> points{};
    for (int f=0;f<5;++f) for (int k=0;k<4;++k) {
        const int i=3*(1+4*f+k); points[i]=.02*(f-2); points[i+1]=.02*(k+1); points[i+2]=.002*k;
    }
    assert(tianji_hand_optimizer_state(solver,nullptr,state.data(),0)==1);
    assert(tianji_hand_optimizer_evaluate(solver,q.data(),points.data(),63,nullptr,evaluated.data())==0);
    assert(tianji_hand_optimizer_solve(solver,points.data(),63,nullptr,q.data())>=0);
    assert(tianji_hand_optimizer_state(solver,nullptr,state.data(),0)==0 && state==q);
    state.fill(42);
    points[62]=std::numeric_limits<double>::quiet_NaN();
    assert(tianji_hand_optimizer_solve(solver,points.data(),63,nullptr,state.data())==-1);
    for (double x:state) assert(x==42);
    assert(tianji_hand_optimizer_state(solver,q.data(),nullptr,1)==-1);
    assert(tianji_hand_optimizer_state(solver,nullptr,state.data(),0)==0 && state==q);
    int foreign=0;
    std::thread other([&] {foreign=tianji_hand_optimizer_state(solver,nullptr,state.data(),0);});
    other.join(); assert(foreign==-1);
    assert(tianji_hand_optimizer_state(solver,nullptr,nullptr,1)==1);
    tianji_hand_optimizer_destroy(solver);
}
