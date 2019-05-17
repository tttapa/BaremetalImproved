
# Creating Autopilot_AMP_Improved

Launch SDK

Window>Preferences
--- step 1 ---
Xilinx SDK>Toolchain Preferences, change...
"Cortex-a9" to "Third Party"
"Toolchain Path" to "~/opt/gcc-arm-8.3-2019.03-x86_64-arm-eabi/bin"
"Toolchain Command Prefix" to "arm-eabi-"
Hit OK

--- step 2 ---
C/C++>Build>Global Tools Paths, change...
"Default toolchain" to "Custom"
"Build tools folder" to "~/opt/gcc-arm-8.3-2019.03-x86_64-arm-eabi"
"Toolchain folder" to "~/opt/gcc-arm-8.3-2019.03-x86_64-arm-eabi/bin"


File>New>Application Project
Name "Autopilot_AMP_Improved"
Change "Processor" to "ps7_cortexa9_1"
Change "Language" to "C++"
Hit Next
Hit Finish

Open Autopilot_AMP_Improved_bsp>system.mss
Modify this BSP's settings
In Overview>drivers>ps7_cortexa9_1 append " -DUSE_AMP=1" to "extra_compiler_flags" in the value column
Hit OK

Open "Autopilot_AMP_Improved_bsp/ps7_cortexa9_1/libsrc/standalone_v5_5/src/kill.c"
On line 46 change it to "int kill(pid_t pid, int sig)"

Create symlinks to src-vivado and install-vivado
delete "Autopilot_AMP_Improved/src" folder
ln -s "path/to/src-vivado" "path/to/Autopilot_AMP_Improved/src"
ln -s "path/to/install-vivado" "path/to/Autopilot_AMP_Improved/vivado-libs"

Right-click Autopilot_AMP_Improved project and select C/C++ build settings
--- step 1 ---
Under C/C++ Build>Settings in the tab "Tool Settings" click on "Target Processor"
Change "ARM Family" to "cortex-a9"
? The rest should be Toolchain default ? <-- check on my pc
Change "Float ABI" to "FP instructions (hard)"
Change "FPU Type" to vfpv3

--- step 2 ---
Under C/C++ Build>Settings in the tab "Tool Settings" click on "Optimization"
Change "Optimization level" to "Optimized most (-O3)"
Check the following warnings: "(-Wunused)", "(-Wuninitialised)", "(-Wall)", "(-Wextra)", "(-Werror)"
Enter the following into "Other warning flags": "-Wno-unknown-pragmas"

--- step 3 --- 
Under C/C++ Build>Settings in the tab "Tool Settings" click on "Cross ARM C++ Compiler>Preprocessor"
Click on plus icon by "Defined symbols (-D)" pane and add "ZYBO" and then add "BAREMETAL"

--- step 4 --- 
Under C/C++ Build>Settings in the tab "Tool Settings" click on "Cross ARM C++ Compiler>Includes"
Click on plus icon by "Include paths (-l)" pane, click "Workspace" and then select "Autopilot_AMP_Improved/src/include"
Click on plus icon by "Include paths (-l)" pane, click "Workspace" and then select "Autopilot_AMP_Improved/vivado-libs/include"

--- step 5 --- 
Under C/C++ Build>Settings in the tab "Tool Settings" click on "Cross ARM C++ Compiler>Optimization"
Change "Language Standard" to "Toolchain default"
Add following flag to "Other optimization flags": "-std=c++17"

--- step 6 --- 
Under C/C++ Build>Settings in the tab "Tool Settings" click on "Cross ARM C++ Linker>Libraries"
Click on plus icon by "Libraries (-l)" pane, enter "main" then click OK. Do the same for "instances", "control", "misc", "time", "communication" IN THAT ORDER!
Click on plus icon by "Library search path (-L)" pane, click "Workspace" and then select "Autopilot_AMP_Improved/vivado-libs/lib" then click OK
