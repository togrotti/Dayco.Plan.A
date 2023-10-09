
set tcl_path      [file dirname [info script]]

# Configure workspace (should be outside of the Git mapping)
set workspace     ${tcl_path}/../zynq_axndc

# Set platform and project names
set hw_project {"zynq_axndc_7z010i2"}
set xsa_file   {"./xsa/zynq_axndc_7z010i2.xsa"}

set app_project   "axndc"

# Configure path for sources, xsa file, setting, launch
set app_sources   [file join ${tcl_path} src]
set app_script    [file join ${tcl_path} script]
set app_setting   [file join ${tcl_path} setting ${app_project}]
set app_launch    [file join ${tcl_path} launch ${app_project}]

#
set repo_path ../axxembedsw

proc setActive { app_name configuration } {
   app config -name ${app_name} build-config ${configuration}
}

proc setBuildOptions { app_name configuration } {
   setActive ${app_name} ${configuration}
   # set the include path option
   app config -name ${app_name} include-path \"$\{workspace_loc:/$\{ProjName\}/src\}\"
   # set math library
   app config -name ${app_name} -add libraries m
   # generate map file
   app config -name ${app_name} -add linker-misc -Wl,-Map=${app_name}.map 
   # keep the preprocessed file (*.i)
   app config -name ${app_name} -add compiler-misc -save-temps=obj
}

proc setSymbol { app_name configuration symbol} {
   setActive ${app_name} ${configuration}
   # set symbols
   app config -name ${app_name} -add define-compiler-symbols ${symbol}
}

# Set workspace 
setws -switch ${workspace}

# Set custom repository
repo -set ${repo_path}

# Generate all platforms
foreach i $hw_project j $xsa_file {
	# Import hardware platform
	platform create -name $i -hw $j

	# Add compiler definitions for fsbl application
	platform config -extra-compiler-flags fsbl "-O2 -MMD -MP -mcpu=cortex-a9 -mfpu=vfpv3 -mfloat-abi=hard  -DDDRLESS_SYSTEM -DFSBL_DEBUG_INFO"

	puts "creating FreeRTOS domain...";
	domain create -name "freertos" -os freertos10_xilinx -proc ps7_cortexa9_0 
	domain active freertos

	# Add custom freertos
	bsp setosversion -ver 1.11
	bsp config use_tick_hook true
	#decrease the stack and heap size, changed 2022-12-21
	bsp config total_heap_size     17360 
	#####
	bsp config tick_rate     1000
	bsp config use_task_fpu_support 2
	bsp config use_fpu_safe_irq_handler true
	bsp config stdin none
	bsp config stdout none
	bsp regenerate

	platform generate
}

# Create app project as empty C++ application
app create -name ${app_project} -domain freertos -lang C -template "Empty Application"

# Set the build options for both release and debug configurations
setBuildOptions ${app_project} release;
setSymbol       ${app_project} release USE_STDINT_FOR_MISRA_C
setSymbol       ${app_project} release ALPLC_P_ARM9;
setSymbol       ${app_project} release ALPLC_P_ARM32;
setSymbol       ${app_project} release ALPLC_C_GCCARM9;
setSymbol       ${app_project} release _APP_XC;
setSymbol       ${app_project} release _AXX_SYSAPP;
setSymbol       ${app_project} release _HW_DC;

setBuildOptions ${app_project} debug;
setSymbol       ${app_project} debug USE_STDINT_FOR_MISRA_C
setSymbol       ${app_project} debug ALPLC_P_ARM9;
setSymbol       ${app_project} debug ALPLC_P_ARM32;
setSymbol       ${app_project} debug ALPLC_C_GCCARM9;
setSymbol       ${app_project} debug _APP_XC;
setSymbol       ${app_project} debug _AXX_SYSAPP;
setSymbol       ${app_project} debug _HW_DC;
setSymbol       ${app_project} debug _APP_DEBUG;

# Set the active build configuration to debug.
setActive ${app_project} debug;

# Import sources from ${app_sources} (including linker script) 
importsources -name ${app_project} -path ${app_sources} -linker-script 

# Import script from ${app_script}
importsources -name ${app_project} -path ${app_script} -target-path .script\

# Import setting from ${app_setting}
importsources -name ${app_project} -path ${app_setting} -target-path .\

# Import launch from ${app_launch}
#importsources -name ${app_project} -path ${app_launch} -target-path .launch\
