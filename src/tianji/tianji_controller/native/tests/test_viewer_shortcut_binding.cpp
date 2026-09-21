// No window, GL context, input devices or controller. Validate the actual
// MuJoCo UI parser used by Simulate::MakeRenderingSection (3.10.0).
#include <mujoco/mujoco.h>
#include <gtest/gtest.h>
#include <cstdio>
#include <memory>

namespace {
int makeConvexHullShortcut() {
  auto ui=std::make_unique<mjUI>();
  mjtByte enabled=0;
  mjuiDef def[] = {
      {mjITEM_SECTION,"Rendering",1,nullptr,"",0},
      {mjITEM_CHECKBYTE,"Convex Hull",2,&enabled,"",0},
      {mjITEM_END,"",0,nullptr,"",0}};
  const char* shortcut=mjVISSTRING[mjVIS_CONVEXHULL][2];
  if(shortcut[0])std::snprintf(def[1].other,sizeof(def[1].other)," %s",shortcut);
  mjui_add(ui.get(),def);
  return ui->sect[0].item[0].single.shortcut;
}

TEST(ViewerShortcutBinding, RemovingExportedShortcutBeforeUiBuildDisablesH) {
  ASSERT_STREQ(mj_versionString(),"3.10.0");
  EXPECT_EQ(makeConvexHullShortcut(),'H');
  const char* original=mjVISSTRING[mjVIS_CONVEXHULL][2];
  mjVISSTRING[mjVIS_CONVEXHULL][2]="";
  const int suppressed=makeConvexHullShortcut();
  mjVISSTRING[mjVIS_CONVEXHULL][2]=original;
  EXPECT_EQ(suppressed,0);
  EXPECT_EQ(makeConvexHullShortcut(),'H');
}
}  // namespace
