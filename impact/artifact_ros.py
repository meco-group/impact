# https://github.com/ramsafin/catkin-package-template
import casadi


def escape(e): # From mpc.py in Impact
  return e.replace('\\','/')


def format_float(e):
    return "%0.18f" % e


def strlist(a):
      elems = []
      for e in a:
        if isinstance(e,str):
          elems.append('"'+e+'"')
        elif isinstance(e,float):
          elems.append(format_float(e))
        else:
          elems.append(str(e))
      return ",".join(elems)


class ROS2Generator:
    """This should be a description of the ROS2Generator class.
    It's common for programmers to give a code example inside of their
    docstring::

        ROS2gen = ROS2Generator(
                mpc_specification = mpc,
                package_name = name,
                ros2_options= ros2_options,
        )

    Here is a link to :py:meth:`__init__`.
    """

    def __init__(self, mpc_specification, ros2_options, **kwargs):
        """Initialize ROS package generator for Impact MPC object

        :param mpc_specification: Impact MPC object
        :type name: MPC

        :param ros2_options: Options regarding the generated ROS 2 executables
        :type ros2_options: dictionary

        :param package_name: Name of the ROS 2 package
        :type package_name: string

        :param version: Version of the ROS 2 package
        :type version: string

        :param description: Description of the ROS 2 package
        :type description: string

        :param maintainer: Information of the maintainer of the ROS 2 package
        :type maintainer: dictionary

        :param license: License of the ROS 2 package
        :type license: string

        :param author: Information of the author of the ROS 2 package
        :type author: dictionary

        :param impact_node_name: Name of the Impact node in the ROS 2 package
        :type impact_node_name: string
        """

        self.mpc_specification = mpc_specification
        self.ros2_options = ros2_options

        self.values_dict = {
            'PKG_NAME' : kwargs['package_name'] if 'package_name' in kwargs else "impact_mpc",
            'PKG_VERSION' : kwargs['version'] if 'version' in kwargs else "0.0.0",
            'PKG_DESCRIPTION' : kwargs['description'] if 'description' in kwargs else "This is a ROS 2 package to execute Impact MPC.",
            'MAINTAINER_EMAIL' : kwargs['maintainer']['email'] if ('maintainer' in kwargs and 'email' in kwargs['maintainer']) else "no@maintainer.com",
            'MAINTAINER_NAME' : kwargs['maintainer']['name'] if ('maintainer' in kwargs and 'name' in kwargs['maintainer']) else "no maintainer",
            'LICENSE': kwargs['license'] if 'license' in kwargs else "None",
            'AUTHOR_NAME': kwargs['author']['name'] if ('author' in kwargs and 'name' in kwargs['author']) else "no author",
            'AUTHOR_EMAIL': kwargs['author']['email'] if ('author' in kwargs and 'email' in kwargs['author']) else "no@author.com",
        }

        self.name = self.values_dict['PKG_NAME']
        
        # --- Default sampling time from OCP: Ts = T / N (robust) ---
        from casadi import DM, MX

        # --- Default sampling time from OCP: Ts = T / N (CasADi-native) ---
        T = getattr(self.mpc_specification, "T", None)
        method = getattr(self.mpc_specification, "_method", None)
        N = getattr(method, "N", None)

        T_num = None

        # direct numeric
        if isinstance(T, (int, float)):
            T_num = float(T)
        # DM scalar -> float
        elif isinstance(T, DM) and T.is_scalar():
            T_num = float(T)
        # constant MX -> DM -> float
        elif isinstance(T, MX) and T.is_constant():
            T_num = float(DM(T))
        # assigned numeric value (DM) -> float
        elif T is not None:
            try:
                T_num = float(self.mpc_specification.value(T))
            except Exception:
                try:
                    T_num = float(self.mpc_specification.initial_value(T))
                except Exception:
                    pass

        if isinstance(N, int) and N > 0 and T_num is not None:
            self.Ts_default = T_num / N
        else:
            # No numeric T available at generation time -> fallback
            self.Ts_default = 0.1


        # dependencies = ['roscpp', '']

    def generate_package_xml(self):

        package_xml_code = """<?xml version="1.0"?>
<?xml-model href="http://download.ros.org/schema/package_format3.xsd" schematypens="http://www.w3.org/2001/XMLSchema"?>
<package format="3">
    <name>{PKG_NAME}</name>
    <version>{PKG_VERSION}</version>
    <description>{PKG_DESCRIPTION}</description>

    <!-- One maintainer tag required, multiple allowed, one person per tag -->
    <maintainer email="{MAINTAINER_EMAIL}">{MAINTAINER_NAME}</maintainer>

    <!-- One license tag required, multiple allowed, one license per tag -->
    <license>{LICENSE}</license>
 
        """.format(**self.values_dict)


        if "REPOSITORY_URL" in self.values_dict:
            package_xml_code += """
    <!-- Url tags are optional, but multiple are allowed, one per tag -->
    <!-- Optional attribute type can be: website, bugtracker, or repository -->
    <url type="repository">{REPOSITORY_URL}</url>

            """.format(**self.values_dict)

        if all(key in self.values_dict for key in ("AUTHOR_EMAIL", "AUTHOR_NAME")):
            package_xml_code += """
    <!-- Author tags are optional, multiple are allowed, one per tag -->
    <!-- Authors do not have to be maintainers, but could be -->
    <author email="{AUTHOR_EMAIL}">{AUTHOR_NAME}</author> 
        
            """.format(**self.values_dict)

        package_xml_code += """
    <buildtool_depend>ament_cmake</buildtool_depend>

    <depend>rclcpp</depend>
    <depend>std_msgs</depend>

    <test_depend>ament_lint_auto</test_depend>
    <test_depend>ament_lint_common</test_depend>

    <!-- The export tag contains other, unspecified, tags -->
    <export>
        <build_type>ament_cmake</build_type>
    </export>
</package>
        """

        return package_xml_code

    def generate_CMakeLists(self):

        self.values_dict['CASADI_PATH'] = escape(casadi.GlobalOptions.getCasadiPath())

        SOLVER_LIBRARY = ''
        INCLUDE_DIRS = f"include include/{self.values_dict['PKG_NAME']}"
        if self.SOLVER_METHOD == 'ipopt':
            SOLVER_LIBRARY = 'ipopt'
            INCLUDE_DIRS += f' ${{CASADI_INCLUDEDIR}} ${{CASADI_INCLUDEDIR}}/include'
        elif self.SOLVER_METHOD == 'fatrop-rockit':
            SOLVER_LIBRARY = f'${{CMAKE_SOURCE_DIR}}/src/libfatrop_driver.so'
        elif self.SOLVER_METHOD == 'fatrop': # From CasADi
            SOLVER_LIBRARY = f'${{CASADI_DIR}}/libfatrop.so'
            INCLUDE_DIRS += f' ${{CASADI_INCLUDEDIR}} ${{CASADI_INCLUDEDIR}}/include'

        self.values_dict['INCLUDE_DIRS'] = INCLUDE_DIRS
        self.values_dict['SOLVER_LIBRARY'] = SOLVER_LIBRARY 


        cmakelists_code = """cmake_minimum_required(VERSION 3.5 FATAL_ERROR)
project({PKG_NAME})

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(CMAKE_C_FLAGS "${{CMAKE_C_FLAGS}} -g -fPIC -pedantic -Wall -Wextra -Wno-unknown-pragmas -Wno-long-long -Wno-unused-parameter -Wno-unused-const-variable -Wno-sign-compare -Wno-unused-but-set-variable -Wno-unused-variable -Wno-endif-labels")

#######################
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(builtin_interfaces REQUIRED)
        """.format(**self.values_dict)

        if self.SOLVER_METHOD == 'ipopt' or self.SOLVER_METHOD == 'fatrop':
            cmakelists_code += """

#######################
## Find CasADi
#######################
message("If you get a complaint about missing libcasadi.lib, please copy the casadi.lib file to libcasadi.lib")
message("Please check that the CASADI_DIR variable in the CMakeLists.txt corresponds to the path of your CasADi installation")
set(CASADI_DIR "{CASADI_PATH}")
set(CMAKE_PREFIX_PATH ${{CASADI_DIR}} ${{CMAKE_PREFIX_PATH}})
find_package(casadi 
  REQUIRED 
  HINTS ${{CASADI_DIR}} ${{CASADI_DIR}}/lib
)
set(CASADI_INCLUDEDIR ${{CASADI_DIR}}/include)
set(CASADI_LIBRARIES casadi)
        """.format(**self.values_dict)

        if self.SOLVER_METHOD == 'ipopt':
            cmakelists_code += """

#######################
## Find Ipopt
#######################
## Option 1:
## Test if pkg-config is able to find your Ipopt installation with the following command: pkg-config --libs ipopt 
## You can install ipopt in Ubuntu by using: sudo apt-get install coinor-libipopt-dev

message("If you get a complaint about missing IPOPT, please install IPOPT with 'sudo apt-get install coinor-libipopt-dev' for Ubuntu")
find_package(PkgConfig)
pkg_search_module(IPOPT REQUIRED ipopt)

## Option 2:
# find_package(ipopt 
#   REQUIRED 
#   HINTS /usr/lib
# )
        """.format(**self.values_dict)

        cmakelists_code += """

###########
## Build ##
###########

# Build libraries and executable
include_directories(
    {INCLUDE_DIRS}
)

## IMPACT C library and codegen
add_library({PKG_NAME}_codegen SHARED src/{PKG_NAME}_codegen.c)
target_link_libraries({PKG_NAME}_codegen 
    m 
    {SOLVER_LIBRARY}
)

add_library({PKG_NAME} SHARED src/{PKG_NAME}.c)
target_link_libraries({PKG_NAME} 
    {PKG_NAME}_codegen 
    {SOLVER_LIBRARY}
)

## Declare a C++ executable.
add_executable({PKG_NAME}_controller src/{PKG_NAME}_controller.cpp)

## Specify libraries to link an executable target against.
ament_target_dependencies({PKG_NAME}_controller
    rclcpp
    std_msgs
    builtin_interfaces
)
target_link_libraries({PKG_NAME}_controller 
    {PKG_NAME}
    {PKG_NAME}_codegen
)

ament_export_libraries(
  {PKG_NAME}
  {PKG_NAME}_codegen
)

install(TARGETS
    {PKG_NAME}_controller
    DESTINATION lib/${{PROJECT_NAME}})



    

# --- Discrete model node (extra) ---
if(EXISTS "${{CMAKE_CURRENT_SOURCE_DIR}}/src/{PKG_NAME}_model.c" AND
   EXISTS "${{CMAKE_CURRENT_SOURCE_DIR}}/src/{PKG_NAME}_model.cpp")
  add_executable({PKG_NAME}_model
    src/{PKG_NAME}_model.cpp
    src/{PKG_NAME}_model.c)
  target_include_directories({PKG_NAME}_model PUBLIC {INCLUDE_DIRS})
  ament_target_dependencies({PKG_NAME}_model rclcpp std_msgs builtin_interfaces)
  install(TARGETS {PKG_NAME}_model DESTINATION lib/${{PROJECT_NAME}})
endif()








install(TARGETS
    {PKG_NAME}
    {PKG_NAME}_codegen
    DESTINATION lib)

        """.format(**self.values_dict)

        if self.SOLVER_METHOD == 'fatrop':
            cmakelists_code += """
install(FILES
    ${{CASADI_DIR}}/libblasfeo.so
    ${{CASADI_DIR}}/libfatrop.so
    DESTINATION lib)
            """.format(**self.values_dict)

        elif self.SOLVER_METHOD == 'fatrop-rockit':
            cmakelists_code += """
install(FILES
    ${{CMAKE_SOURCE_DIR}}/src/libfatrop_driver.so
    ${{CMAKE_SOURCE_DIR}}/src/libfatrop.so
    DESTINATION lib)
            """.format(**self.values_dict)

        cmakelists_code += """

ament_package()

        """.format(**self.values_dict)

        return cmakelists_code

    def generate_controller_node_cpp(self):
        name = self.name
        ros2_class_name = "Controller"+((''.join(e for e in name if e.isalnum())).capitalize())+"Node"
        ts_default_str = f"{self.Ts_default:.16g}"
        
        cpp_node_code = f"""#include <chrono>
#include <vector>
#include <stdio.h>
#include <stdlib.h>
#include <assert.h>
extern "C"{{
  #include <{name}.h>
}}
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float32.hpp"
#include "std_msgs/msg/float32_multi_array.hpp"
#include "std_msgs/msg/empty.hpp"
#include "std_msgs/msg/bool.hpp"


class {ros2_class_name} : public rclcpp::Node
{{
  public:
    {ros2_class_name}()
      : Node("controller_{name}_node"),
        m(impact_initialize(0, 0)),
        control_enabled(true),
        u_scratch(nullptr),
        x_scratch(nullptr),
        u_initial_guess(nullptr),
        x_initial_guess(nullptr),
        hotstart(1.0),
        n_row_x(0),
        n_col_x(0),
        n_row_u(0),
        n_col_u(0)"""      
      
        if self.mpc_specification.parameters['']:
            for param in self.mpc_specification.parameters['']:
                if isinstance(self.mpc_specification.initial_value(self.mpc_specification.value(param)),float):
                    cpp_node_code += f""",  
        val_{param.name()}{{{str(self.mpc_specification.initial_value(self.mpc_specification.value(param)))}}}"""
                else:
                    cpp_node_code += f""",  
        val_{param.name()}{{{strlist(self.mpc_specification.initial_value(self.mpc_specification.value(param)))}}}"""

        cpp_node_code += f""" 
    {{
        if (!m) {{
            RCLCPP_ERROR(this->get_logger(), "Failed to initialize");
            rclcpp::shutdown();
            return;
        }}"""

        if self.mpc_specification.parameters['']:
            for param in self.mpc_specification.parameters['']:
                cpp_node_code += f"""  
        impact_set(m, "p", "{param.name()}", IMPACT_EVERYWHERE, val_{param.name()}, IMPACT_FULL);"""

        cpp_node_code += f""" 
        
        impact_set(m, "x_current", IMPACT_ALL, IMPACT_EVERYWHERE, val_x_current, IMPACT_FULL);

        impact_print_problem(m);

        flag = impact_solve(m);
        if (flag) {{
            RCLCPP_ERROR(this->get_logger(), "Solve indicates a problem: return code %d.", flag);
        }}
        RCLCPP_INFO(this->get_logger(), "Solve finished.");

        stats = impact_get_stats(m);
        if (stats) {{
            printf("Runtime [s]: %e\\n\\n", stats->runtime);
        }} else {{
            printf("No stats available.\\n");
        }}

        impact_print_problem(m);

        set_initial_guess();

        impact_print_problem(m);

        impact_solve(m);


        /* Allocate scratch space for state and control trajectories */
        impact_get_size(m, "u_opt", IMPACT_ALL, IMPACT_EVERYWHERE, IMPACT_FULL, &n_row_u, &n_col_u);
        RCLCPP_INFO(this->get_logger(), "u_opt dims: %d - %d", n_row_u, n_col_u);
        u_scratch = (double *)malloc(sizeof(double)*n_row_u);

        impact_get_size(m, "x_opt", IMPACT_ALL, IMPACT_EVERYWHERE, IMPACT_FULL, &n_row_x, &n_col_x);
        RCLCPP_INFO(this->get_logger(), "x_opt dims: %d - %d", n_row_x, n_col_x);
        x_scratch = (double *)malloc(sizeof(double)*n_row_x*n_col_x);

        /* Allocate scratch space for state and control initial guesses */
        impact_get_size(m, "u_initial_guess", IMPACT_ALL, IMPACT_EVERYWHERE, IMPACT_FULL, &n_row_u, &n_col_u);
        u_initial_guess = (double *)malloc(sizeof(double)*n_row_u*n_col_u);

        impact_get_size(m, "x_initial_guess", IMPACT_ALL, IMPACT_EVERYWHERE, IMPACT_FULL, &n_row_x, &n_col_x);
        x_initial_guess = (double *)malloc(sizeof(double)*n_row_x*n_col_x);


        /* Solve OCP once */
        RCLCPP_INFO(this->get_logger(), "Solving Single OCP");

        impact_get(m, "x_opt", IMPACT_ALL, IMPACT_EVERYWHERE, x_scratch, IMPACT_FULL);
        //for (i=0;i<n_col_x;++i) {{
        //    for (j=0;j<n_row_x;++j) {{
        //        printf("%0.3e ", x_scratch[j+i*n_row_x]);
        //    }}
        //    printf("\\n");
        //}}

        free(x_scratch);
        x_scratch = (double *)malloc(sizeof(double)*n_row_x*n_col_x);

        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "IMPACT library initialization done");

        // Create a subscription to the "/impact/control_enable" topic
        control_enable_subscriber_ = this->create_subscription<std_msgs::msg::Bool>(
           "/impact/control_enable",
           10,
           std::bind(&{ros2_class_name}::control_enable_callback, this, std::placeholders::_1)
        );
        // Create a subscription to the "/impact/x_initial_guess" topic
        x_initial_guess_subscriber_ = this->create_subscription<std_msgs::msg::Float32MultiArray>(
            "/impact/x_initial_guess", 
            10, 
            std::bind(&{ros2_class_name}::x_initial_guess_callback, this, std::placeholders::_1)
        );

        // Create a subscription to the "/impact/u_initial_guess" topic
        u_initial_guess_subscriber_ = this->create_subscription<std_msgs::msg::Float32MultiArray>(
            "/impact/u_initial_guess", 
            10, 
            std::bind(&{ros2_class_name}::u_initial_guess_callback, this, std::placeholders::_1)
        );

        // Create a subscription to the "/impact/hotstart" topic
        hotstart_subscriber_ = this->create_subscription<std_msgs::msg::Float32>(
            "/impact/hotstart", 
            10, 
            std::bind(&{ros2_class_name}::hotstart_callback, this, std::placeholders::_1)
        );
        // Create a subscription to the "/impact/reset_hotstart" topic
        reset_hotstart_subscriber_ = this->create_subscription<std_msgs::msg::Empty>(
            "/impact/reset_hotstart", 
            10, 
            std::bind(&{ros2_class_name}::reset_hotstart_callback, this, std::placeholders::_1)
        );"""

        if self.mpc_specification.parameters['']:
            for param in self.mpc_specification.parameters['']:
                cpp_node_code += f"""
        // Create a subscription to the "impact/{param.name()}" topic  
        {param.name()}_subscriber_ = this->create_subscription<std_msgs::msg::Float32MultiArray>(
            "/impact/{param.name()}", 
            10, 
            std::bind(&{ros2_class_name}::{param.name()}_callback, this, std::placeholders::_1)
        );
        """

        cpp_node_code += f""" 
        x_opt_publisher_ = this->create_publisher<std_msgs::msg::Float32MultiArray>("/impact/x_opt", 10);

        control_to_apply_publisher_ = this->create_publisher<std_msgs::msg::Float32MultiArray>("/impact/control_to_apply", 10);

        enabled_status_publisher = this->create_publisher<std_msgs::msg::Bool>("/impact/control_enabled_status", 10);
        
        
        
        
        
        
        
        
        // --- Ts parameter (default = T/N from the OCP) ---
        this->declare_parameter<double>("Ts", {ts_default_str});
        double Ts_ = this->get_parameter("Ts").as_double();
        
        timer_ = this->create_wall_timer(
            std::chrono::duration<double>(Ts_),
            std::bind(&{ros2_class_name}::execute, this));
        
        

        
        
        
        
        
        
    }}
    bool x_current_updated = false;
    bool x_initial_guess_updated = false;
    bool u_initial_guess_updated = false;

  private:

    void execute()
    {{
        if (x_initial_guess_updated) {{ // Update initial guess of x
          impact_set(m, "x_initial_guess", IMPACT_ALL, IMPACT_EVERYWHERE, x_initial_guess, IMPACT_FULL);
          x_initial_guess_updated = false;
        }}

        if (u_initial_guess_updated) {{ // Update initial guess of u
          impact_set(m, "u_initial_guess", IMPACT_ALL, IMPACT_EVERYWHERE, u_initial_guess, IMPACT_FULL);
          u_initial_guess_updated = false;
        }}

        std_msgs::msg::Bool control_enabled_status;
        control_enabled_status.data = control_enabled; 
        enabled_status_publisher->publish(control_enabled_status);

        if (!x_current_updated or !control_enabled) {{ // Only execute if x_current has been updated and control is enabled
            return;
        }}

        
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Executing");

        // Set parameters"""

        if self.mpc_specification.parameters['']:
            for param in self.mpc_specification.parameters['']:
                cpp_node_code += f"""  
        impact_set(m, "p", "{param.name()}", IMPACT_EVERYWHERE, val_{param.name()}, IMPACT_FULL);"""

        cpp_node_code += f""" 
        
        // Do hotstart / freshstart / coldstart
        set_initial_guess();
        
        // Trigger OCP solve
        // impact_solve(m);
        flag = impact_solve(m);
        if (flag) {{"""

        if 'repeat_on_fail' in self.ros2_options and self.ros2_options['repeat_on_fail']:
            cpp_node_code += f"""
            RCLCPP_ERROR(this->get_logger(), "Solve indicates a problem: return code %d. Solving again with coldstart", flag);
            impact_coldstart(m);
            flag = impact_solve(m);
            if (flag) {{
                RCLCPP_ERROR(this->get_logger(), "Solve indicates a problem, even after coldstart: return code %d.", flag);
            }}
            else {{
                RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Successfully solved after cold-starting.");
                if (hotstart == 1.0) {{
                    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Switching to hotstart for the subsequent executions.");
                    impact_hotstart(m);
                }} else if (hotstart == -1.0) {{
                    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Switching to freshstart for the subsequent executions.");
                    impact_freshstart(m);
                }} else if (hotstart == 0.0) {{
                    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Keeping coldstart for the subsequent executions.");
                    impact_coldstart(m);
                }}
            }}"""
        else:
            cpp_node_code += f"""
            RCLCPP_ERROR(this->get_logger(), "Solve indicates a problem: return code %d.", flag);"""     

        cpp_node_code += f"""
        }}
        
        // Get stats
        stats = impact_get_stats(m);
        if (stats) {{
            printf("Runtime [s]: %e\\n\\n", stats->runtime);
        }} else {{
            printf("No stats available.\\n");
        }}

        // Get solution
        impact_get(m, "u_opt", IMPACT_ALL, 0, u_scratch, IMPACT_FULL);
        impact_get(m, "x_opt", IMPACT_ALL, IMPACT_EVERYWHERE, x_scratch, IMPACT_FULL);

        //for (i=0;i<n_col_x;++i) {{
        //    for (j=0;j<n_row_x;++j) {{
        //        printf("%0.3e ", x_scratch[j+i*n_row_x]);
        //    }}
        //    printf("\\n");
        //}}
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Execution done");

        // Publish x_scratch values
        std_msgs::msg::Float32MultiArray x_next_msg;
        x_next_msg.data.insert(x_next_msg.data.end(), x_scratch, x_scratch + n_row_x * n_col_x);
        x_opt_publisher_->publish(x_next_msg);

        // Publish u_scratch values
        std_msgs::msg::Float32MultiArray control_to_apply_msg;
        control_to_apply_msg.data.insert(control_to_apply_msg.data.end(), u_scratch, u_scratch + n_row_u);
        control_to_apply_publisher_->publish(control_to_apply_msg);

        //free(u_scratch);
        //free(x_scratch);

        //impact_destroy(m);

        x_current_updated = false;

    }}


    void control_enable_callback(const std_msgs::msg::Bool::SharedPtr msg)
    {{
        if (msg->data) {{
            control_enabled = true;
            RCLCPP_INFO(this->get_logger(), "Control enabled");
        }} else {{
            control_enabled = false;
            RCLCPP_INFO(this->get_logger(), "Control disabled");
        }}
    }}


    void set_initial_guess(){{ 
        if (hotstart == 1.0) {{
            impact_hotstart(m);
        }} else if (hotstart == -1.0) {{
            impact_freshstart(m);
        }} else if (hotstart == 0.0) {{
            impact_coldstart(m);
        }}
    }}


    void set_initial_guess(double hotstart_setting){{ 
        if (hotstart_setting == 1.0) {{
            impact_hotstart(m);
        }} else if (hotstart_setting == -1.0) {{
            impact_freshstart(m);
        }} else if (hotstart_setting == 0.0) {{
            impact_coldstart(m);
        }}
    }}


    void reset_hotstart(){{
        // Set parameters
"""
        if self.mpc_specification.parameters['']:
            for param in self.mpc_specification.parameters['']:
                cpp_node_code += f"""        impact_set(m, "p", "{param.name()}", IMPACT_EVERYWHERE, val_{param.name()}, IMPACT_FULL);
"""
        cpp_node_code += f"""
        // Do freshstart
        set_initial_guess(-1.0);

        // Trigger OCP solve
        // impact_solve(m);
        flag = impact_solve(m);
        if (flag) {{
            RCLCPP_ERROR(this->get_logger(), "Solve indicates a problem: return code %d.", flag);
        }}

        RCLCPP_INFO(this->get_logger(), "Hotstart initial guess reset");

    }}

    void reset_hotstart_callback(const std_msgs::msg::Empty::SharedPtr /*msg*/)
    {{
        reset_hotstart();
    }}


    void hotstart_callback(const std_msgs::msg::Float32::SharedPtr msg)
    {{
        hotstart = msg->data;
        set_initial_guess();
        RCLCPP_INFO(this->get_logger(), "Hotstart set to: %f", hotstart);
    }}

    
    void x_initial_guess_callback(const std_msgs::msg::Float32MultiArray::SharedPtr msg)
    {{
        //for (int i = 0; i < n_row_x*n_col_x; ++i) {{x_initial_guess[i] = msg->data[i];}}

        for (i=0;i<n_col_x;++i) {{
            for (j=0;j<n_row_x;++j) {{
                x_initial_guess[j+i*n_row_x] = msg->data[j+i*n_row_x];
            }}
        }}

        x_initial_guess_updated = true;
    }}
    
    void u_initial_guess_callback(const std_msgs::msg::Float32MultiArray::SharedPtr msg)
    {{
        //for (int i = 0; i < n_row_u*n_col_u; ++i) {{x_initial_guess[i] = msg->data[i];}}

        for (i=0;i<n_col_u;++i) {{
            for (j=0;j<n_row_u;++j) {{
                u_initial_guess[j+i*n_row_u] = msg->data[j+i*n_row_u];
            }}
        }}

        u_initial_guess_updated = true;
    }}"""

        if self.mpc_specification.parameters['']:
            for param in self.mpc_specification.parameters['']:
                if isinstance(self.mpc_specification.initial_value(self.mpc_specification.value(param)),float):
                    cpp_node_code += f"""
    void {param.name()}_callback(const std_msgs::msg::Float32MultiArray::SharedPtr msg)
    {{
        if (msg->data.size() != 1) {{
            RCLCPP_ERROR(this->get_logger(), "Received parameter array {param.name()} of incorrect size.");
            return;
        }}
        for (int i = 0; i < 1; ++i) {{val_{param.name()}[i] = msg->data[i];}}
        //std::copy_n(msg->data.begin(), 1, val_{param.name()});
        {"x_current_updated = true;" if param.name()=="x_current" else ""}
    }}"""
                else:
                    cpp_node_code += f"""
    void {param.name()}_callback(const std_msgs::msg::Float32MultiArray::SharedPtr msg)
    {{
        if (msg->data.size() != {len(self.mpc_specification.initial_value(self.mpc_specification.value(param)))}) {{
            RCLCPP_ERROR(this->get_logger(), "Received parameter array {param.name()} of incorrect size.");
            return;
        }}
        for (int i = 0; i < {len(self.mpc_specification.initial_value(self.mpc_specification.value(param)))}; ++i) {{val_{param.name()}[i] = msg->data[i];}}
        //std::copy_n(msg->data.begin(), {len(self.mpc_specification.initial_value(self.mpc_specification.value(param)))}, val_{param.name()});
        {"x_current_updated = true;" if param.name()=="x_current" else ""}
    }}"""

        cpp_node_code += f""" 

    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr control_enable_subscriber_;
    rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr hotstart_subscriber_;
    rclcpp::Subscription<std_msgs::msg::Empty>::SharedPtr reset_hotstart_subscriber_;"""
        if self.mpc_specification.parameters['']:
            for param in self.mpc_specification.parameters['']:
                cpp_node_code += f"""
    rclcpp::Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr {param.name()}_subscriber_;"""

        cpp_node_code += f"""
    rclcpp::Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr x_initial_guess_subscriber_;
    rclcpp::Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr u_initial_guess_subscriber_;
    rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr control_to_apply_publisher_;
    rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr x_opt_publisher_;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr enabled_status_publisher;
    impact_struct* m;
    double *u_scratch, *x_scratch;
    double *u_initial_guess, *x_initial_guess;
    double hotstart;
    bool control_enabled;
    const impact_stats* stats;
    int n_row_x, n_col_x;
    int n_row_u, n_col_u;
    int i, j, flag;"""


        if self.mpc_specification.parameters['']:
            for param in self.mpc_specification.parameters['']:
                if isinstance(self.mpc_specification.initial_value(self.mpc_specification.value(param)),float):
                    cpp_node_code += f"""  
    double val_{param.name()}[1];"""
                else:
                    cpp_node_code += f"""  
    double val_{param.name()}[{len(self.mpc_specification.initial_value(self.mpc_specification.value(param)))}];"""

        cpp_node_code += f""" 
}};

int main(int argc, char **argv)
{{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<{ros2_class_name}>());
    rclcpp::shutdown();
    return 0;
}}
  
     
      """
        
        return cpp_node_code
    


    def generate_model_node_cpp(self):
        name = self.name
        ros2_class_name = "Model"+((''.join(e for e in name if e.isalnum())).capitalize())+"Node"
        ts_default_str = f"{self.Ts_default:.16g}"

        return f"""#include <chrono>
#include <vector>
#include <cstdio>
#include <cstdlib>
#include <cassert>
extern "C" {{
  #include <{name}_model.h>   // generated by CasADi: provides {name}_step(...)
}}
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float32.hpp"
#include "std_msgs/msg/float32_multi_array.hpp"

class {ros2_class_name} : public rclcpp::Node {{
public:
  {ros2_class_name}() : Node("model_{name}_node") {{


  



    // T_s parameter (default = T/N from the OCP; override at runtime with -p Ts:=...)
    this->declare_parameter<double>("Ts", {ts_default_str});
    Ts_ = this->get_parameter("Ts").as_double();
    
    
    
   
    sub_x_ = this->create_subscription<std_msgs::msg::Float32MultiArray>(
      "/impact/x_current", 10,
      [this](std_msgs::msg::Float32MultiArray::SharedPtr m){{ x_ = *m; maybe_step(); }});

    sub_u_ = this->create_subscription<std_msgs::msg::Float32MultiArray>(
      "/impact/control_to_apply", 10,
      [this](std_msgs::msg::Float32MultiArray::SharedPtr m){{ u_ = *m; maybe_step(); }});

    pub_xnext_ = this->create_publisher<std_msgs::msg::Float32MultiArray>("/impact/x_next", 10);
  }}

private:
  void maybe_step() {{
    // Expect x.size()==nx and u.size()==nu; you can make these parameters or assert constants
    if (x_.data.empty() || u_.data.empty()) return;

    // --- Query work sizes from CasADi codegen ---
    casadi_int sz_arg=0, sz_res=0, sz_iw=0, sz_w=0;
    {name}_step_work(&sz_arg,&sz_res,&sz_iw,&sz_w);

    std::vector<const double*> arg(sz_arg,nullptr);
    std::vector<double*>       res(sz_res,nullptr);
    std::vector<casadi_int>    iw(sz_iw);
    std::vector<double>        w(sz_w);

    // Allocate and copy inputs
    std::vector<double> x0(x_.data.begin(), x_.data.end());
    std::vector<double> u0(u_.data.begin(), u_.data.end());
    double T = Ts_;
    std::vector<double> xf(x0.size(), 0.0);

    // Bind positional arguments: ["x0","u","T"] -> ["xf"]
    arg[0] = x0.data();
    arg[1] = u0.data();
    arg[2] = &T;
    res[0] = xf.data();

    // Optional memory API (exists in newer CasADi codegen)
    int mem = -1;
    #ifdef {name}_step_alloc_mem
      {name}_step_alloc_mem();
      mem = {name}_step_checkout();
      {name}_step_init_mem(mem);
    #endif

    int flag = {name}_step(arg.data(), res.data(), iw.data(), w.data(), mem);

    #ifdef {name}_step_release
      {name}_step_release(mem);
      {name}_step_free_mem(mem);
    #endif

    if (flag) {{
      RCLCPP_ERROR(this->get_logger(), "{name}_step failed: %d", flag);
      return;
    }}

    std_msgs::msg::Float32MultiArray out;
    out.data.assign(xf.begin(), xf.end());
    pub_xnext_->publish(out);
  }}

  rclcpp::Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr sub_x_, sub_u_;
  rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr pub_xnext_;
  std_msgs::msg::Float32MultiArray x_, u_;
  double Ts_;
}};

int main(int argc, char** argv){{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<{ros2_class_name}>());
  rclcpp::shutdown();
  return 0;
}}
"""













    def export_package(self, directory="ROS2_package"):

        import os
        import shutil

        import rockit
      
        #####################################################
        # Check for solver method: ipopt, sqpmethod, fatrop
        self.SOLVER_METHOD = None

        if hasattr(self.mpc_specification._method, '_solver'):
            self.SOLVER_METHOD = self.mpc_specification._method._solver

        if hasattr(rockit.external, 'fatrop') and isinstance(self.mpc_specification._method, rockit.external.fatrop.method):
            self.SOLVER_METHOD = 'fatrop-rockit' # TODO: Check that this works for other external methods as well

        if self.SOLVER_METHOD is None:
            raise Exception("If you are using an external method, the only allowed method for ROS 2 package generation is Fatrop.")
        ####################################################

        PKG_NAME = self.values_dict['PKG_NAME']

        directory = self.ros2_options['directory'] if 'directory' in self.ros2_options else directory

        ROS_PKG_ROOT = f"{directory}/{PKG_NAME}"

        package_xml_code = self.generate_package_xml()
        cmakelists_code = self.generate_CMakeLists()
        cpp_node_code = self.generate_controller_node_cpp()

        cpp_model_code = self.generate_model_node_cpp()

        if not os.path.exists(ROS_PKG_ROOT):
            os.makedirs(f"{ROS_PKG_ROOT}/src")
            os.makedirs(f"{ROS_PKG_ROOT}/include/{PKG_NAME}")

        # Create package.xml
        with open(f"{ROS_PKG_ROOT}/package.xml", mode='w') as f:
            f.write(package_xml_code)
        # Create CMakeLists.txt
        with open(f"{ROS_PKG_ROOT}/CMakeLists.txt", mode='w') as f:
            f.write(cmakelists_code)
        # Create ROS node from Impact mpc
        with open(f"{ROS_PKG_ROOT}/src/{PKG_NAME}_controller.cpp", mode='w') as f:
            f.write(cpp_node_code)
        

        with open(f"{ROS_PKG_ROOT}/src/{PKG_NAME}_model.cpp", mode='w') as f:
            f.write(cpp_model_code)


        # Get files from build_dir
        build_dir_path = f"{PKG_NAME}_build_dir"
        if os.path.exists(build_dir_path):
            # Copy .c files to source directory
            shutil.copy(f"{build_dir_path}/{PKG_NAME}.c", f"{ROS_PKG_ROOT}/src/{PKG_NAME}.c")
            shutil.copy(f"{build_dir_path}/{PKG_NAME}_codegen.c", f"{ROS_PKG_ROOT}/src/{PKG_NAME}_codegen.c")
            # Copy header files to include directory
            shutil.copy(f"{build_dir_path}/{PKG_NAME}.h", f"{ROS_PKG_ROOT}/include/{PKG_NAME}/{PKG_NAME}.h")
            shutil.copy(f"{build_dir_path}/{PKG_NAME}_codegen.h", f"{ROS_PKG_ROOT}/include/{PKG_NAME}/{PKG_NAME}_codegen.h")
            # Copy header files to include directory
            # shutil.copy(f"{build_dir_path}/{PKG_NAME}_controller.cpp", f"{ROS_PKG_ROOT}/src/{PKG_NAME}_controller.cpp") 



            # --- NEW: model sources ---
            model_c = os.path.join(build_dir_path, f"{PKG_NAME}_model.c")
            model_h = os.path.join(build_dir_path, f"{PKG_NAME}_model.h")
            if os.path.exists(model_c) and os.path.exists(model_h):
                shutil.copy(model_c, f"{ROS_PKG_ROOT}/src/{PKG_NAME}_model.c")
                shutil.copy(model_h, f"{ROS_PKG_ROOT}/include/{PKG_NAME}/{PKG_NAME}_model.h")
            else:
                print("Note: no generated model files found; skipping copy.")
    


            if self.SOLVER_METHOD == 'fatrop-rockit':
                shutil.copy(f"{build_dir_path}/libfatrop_driver.so", f"{ROS_PKG_ROOT}/src/libfatrop_driver.so")
                shutil.copy(f"{build_dir_path}/libfatrop.so", f"{ROS_PKG_ROOT}/src/libfatrop.so")
 


# author = {'name': 'Arthur Impact'}
# maintainer = {'email': 'arthur@maintain.be'}

# ROS2gen = ROS2Generator(name='ROSimpact', version='0.0.3', author=author, maintainer=maintainer)
# ROS2gen.export_package()


class RosGenerator:

    def __init__(self, **kwargs):
        """Initialize ROS package generator for Impact MPC object

        :param name: Name of the ROS package
        :type name: string

        :param version: Version of the ROS package
        :type version: string

        :param description: Description of the ROS package
        :type description: string

        :param maintainer: Information of the maintainer of the ROS package
        :type maintainer: dictionary

        :param license: License of the ROS package
        :type license: string

        :param author: Information of the author of the ROS package
        :type author: dictionary

        :param impact_node_name: Name of the Impact node in the ROS package
        :type impact_node_name: string
        """

        self.values_dict = {
            'PKG_NAME' : kwargs['name'] if 'name' in kwargs else "test_impact",
            'PKG_VERSION' : kwargs['version'] if 'version' in kwargs else "0.0.0",
            'PKG_DESCRIPTION' : kwargs['description'] if 'description' in kwargs else "This is a ROS package to execute Impact MPC.",
            'MAINTAINER_EMAIL' : kwargs['maintainer']['email'] if ('maintainer' in kwargs and 'email' in kwargs['maintainer']) else "no@maintainer.com",
            'MAINTAINER_NAME' : kwargs['maintainer']['name'] if ('maintainer' in kwargs and 'name' in kwargs['maintainer']) else "no maintainer",
            'LICENSE': kwargs['license'] if 'license' in kwargs else "None",
            'AUTHOR_NAME': kwargs['author']['name'] if ('author' in kwargs and 'name' in kwargs['author']) else "no author",
            'AUTHOR_EMAIL': kwargs['author']['email'] if ('author' in kwargs and 'email' in kwargs['author']) else "no@author.com",
            'IMPACT_NODE_NAME': kwargs['impact_node_name'] if 'impact_node_name' in kwargs else "impact_node",
        }

        # dependencies = ['roscpp', '']


    def generate_package_xml(self):

        package_xml_code = """<?xml version="1.0"?>
<?xml-model href="http://download.ros.org/schema/package_format3.xsd" schematypens="http://www.w3.org/2001/XMLSchema"?>
<package format="3">
    <name>{PKG_NAME}</name>
    <version>{PKG_VERSION}</version>
    <description>{PKG_DESCRIPTION}</description>

    <!-- One maintainer tag required, multiple allowed, one person per tag -->
    <maintainer email="{MAINTAINER_EMAIL}">{MAINTAINER_NAME}</maintainer>

    <!-- One license tag required, multiple allowed, one license per tag -->
    <license>{LICENSE}</license>
 
        """.format(**self.values_dict)


        if "REPOSITORY_URL" in self.values_dict:
            package_xml_code += """
    <!-- Url tags are optional, but multiple are allowed, one per tag -->
    <!-- Optional attribute type can be: website, bugtracker, or repository -->
    <url type="repository">{REPOSITORY_URL}</url>

            """.format(**self.values_dict)

        if all(key in self.values_dict for key in ("AUTHOR_EMAIL", "AUTHOR_NAME")):
            package_xml_code += """
    <!-- Author tags are optional, multiple are allowed, one per tag -->
    <!-- Authors do not have to be maintainers, but could be -->
    <author email="{AUTHOR_EMAIL}">{AUTHOR_NAME}</author> 
        
            """.format(**self.values_dict)

        package_xml_code += """
    <buildtool_depend>catkin</buildtool_depend>
    <build_depend>roscpp</build_depend>
    <build_depend>rospy</build_depend>
    <build_depend>std_msgs</build_depend>

    <build_export_depend>roscpp</build_export_depend>
    <build_export_depend>rospy</build_export_depend>
    <build_export_depend>std_msgs</build_export_depend>

    <exec_depend>roscpp</exec_depend>
    <exec_depend>rospy</exec_depend>
    <exec_depend>std_msgs</exec_depend>

    <!-- The export tag contains other, unspecified, tags -->
    <export>
    <!--
        Other tools can request additional information be placed here.

        Example:

        <gazebo gazebo_media_path="${prefix}"/>
        <gazebo_ros gazebo_model_path="${prefix}/models"/>
    -->
    </export>
</package>
        """

        return package_xml_code


    def generate_CMakeLists(self):

        self.values_dict['CASADI_PATH'] = escape(casadi.GlobalOptions.getCasadiPath())

        cmakelists_code = """project({PKG_NAME})

cmake_minimum_required(VERSION 3.15 FATAL_ERROR)

set(CMAKE_WINDOWS_EXPORT_ALL_SYMBOLS ON)

set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_FLAGS "${{CMAKE_CXX_FLAGS}} -w -fPIC -std=c++11")

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

## Find catkin macros and libraries.
## If COMPONENTS list such as find_package(catkin REQUIRED COMPONENTS xyz) is used,
## also find other catkin packages (i.e. populate catkin_* variables).
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
)


## Find CasADi
message("If you get a complaint about missing libcasadi.lib, please copy the casadi.lib file to libcasadi.lib")
set(CASADI_DIR "{CASADI_PATH}")
set(CMAKE_PREFIX_PATH ${{CASADI_DIR}} ${{CMAKE_PREFIX_PATH}})
find_package(casadi 
  REQUIRED 
  HINTS ${{CASADI_DIR}} ${{CASADI_DIR}}/lib
)

## catkin specific configuration.
##
## The catkin_package macro generates cmake config files for your package.
## Declare things to be passed to dependent projects:
##  INCLUDE_DIRS: header files directory
##  LIBRARIES: libraries you create in this project
##  CATKIN_DEPENDS: catkin packages
##  DEPENDS: system dependencies of this project
catkin_package(
  INCLUDE_DIRS include/{PKG_NAME}
#   LIBRARIES ${{PROJECT_NAME}}
#   CATKIN_DEPENDS roscpp rospy std_msgs
#   DEPENDS system_lib
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
  include/{PKG_NAME}
  ${{CASADI_DIR}}/include
  ${{catkin_INCLUDE_DIRS}}
)

## Declare a C++ interface library.
add_library(catkin_deps INTERFACE)
target_link_libraries(catkin_deps INTERFACE ${{catkin_LIBRARIES}})
target_include_directories(catkin_deps INTERFACE include ${{catkin_INCLUDE_DIRS}})

add_library({PKG_NAME}_codegen SHARED src/{PKG_NAME}_codegen.c)
target_link_libraries({PKG_NAME}_codegen casadi m)

add_library({PKG_NAME} SHARED src/{PKG_NAME}.c)
target_link_libraries({PKG_NAME} casadi {PKG_NAME}_codegen)

# ## Declare a C++ library.
# add_library(${{PROJECT_NAME}}
#   src/lib.cpp include/package/lib.hpp
# )

## Specify libraries to link a library target against.
# target_link_libraries(${{PROJECT_NAME}} PUBLIC catkin_deps)

## Specify directories with header files fo the target.
# target_include_directories(${{PROJECT_NAME}} PUBLIC include)

## Add CMake target dependencies of the library.
##
## As an example, code may need to be generated before libraries
## either from message generation or dynamic reconfigure.
# add_dependencies(${{PROJECT_NAME}} ${{${{PROJECT_NAME}}_EXPORTED_TARGETS}} ${{catkin_EXPORTED_TARGETS}})

## Declare a C++ executable.
add_executable({IMPACT_NODE_NAME} src/{IMPACT_NODE_NAME}.cpp)

## Specify libraries to link an executable target against.
target_link_libraries({IMPACT_NODE_NAME}
    {PKG_NAME}
    ${{catkin_LIBRARIES}} 
)


        """.format(**self.values_dict)

        return cmakelists_code


#     def generate_ros_node_cpp(self):

#         cpp_node_code = """#include <ros/ros.h>

# int main (int argc, char **argv)
# {{
#         ros::init(argc, argv, "{IMPACT_NODE_NAME}");
#         ros::NodeHandle nh;
#         ROS_INFO("Node has been started");

#         ros::Rate rate(10);

#         while (ros::ok()) {{
#                 ROS_INFO("Hello");
#                 rate.sleep();
#         }}

# }}

#         """.format(**self.values_dict)

#         return cpp_node_code

    def generate_hello_world_ros_node_cpp(self):

        cpp_node_code = """#include <ros/ros.h>
#include <stdio.h>
#include <stdlib.h>
#include <assert.h>
extern "C"{{
    #include <{PKG_NAME}.h>
}}

int main (int argc, char **argv)
{{
        ros::init(argc, argv, "{IMPACT_NODE_NAME}");
        ros::NodeHandle nh;
        ROS_INFO("Node has been started");

        ros::Rate rate(10);

        int i, j, n_row, n_col, flag;
        double *u_scratch, *x_scratch;
        // impact_struct* m = impact_initialize(printf, 0);
        impact_struct* m = impact_initialize(0, 0);
        if (!m) {{
            ROS_ERROR("Failed to initialize.");
            return 1;
        }}
    
        /* Example: how to set a parameter */
        double val_x_current[14] = {{-1.945999999999999952,1.574999999999999956,2.177000000000000046,-1.620999999999999996,-0.162000000000000005,3.677000000000000046,2.810999999999999943,0.000000000000000000,0.000000000000000000,0.000000000000000000,0.000000000000000000,0.000000000000000000,0.000000000000000000,0.000000000000000000}};
        impact_set(m, "p", "x_current", IMPACT_EVERYWHERE, val_x_current, IMPACT_FULL);

    
        double x0[14] = {{ -1.9460000000000000,1.5750000000000000,2.1770000000000000,-1.6210000000000000,-0.1620000000000000,3.6770000000000000,2.8109999999999999,0.0000000000000000,0.0000000000000000,0.0000000000000000,0.0000000000000000,0.0000000000000000,0.0000000000000000,0.0000000000000000 }};

        impact_set(m, "x_current", IMPACT_ALL, IMPACT_EVERYWHERE, x0, IMPACT_FULL);

        double num = -1.9460000000000000;

        impact_set(m, "x_current", "robot.q1", IMPACT_EVERYWHERE, &num, IMPACT_FULL);

        impact_print_problem(m);


        ROS_INFO("Start a solve.");
        flag = impact_solve(m);
        if (flag) {{
            printf("Solve indicates a problem: return code %d.\\n", flag);
        }}
        ROS_INFO("Solve finished.");

        const impact_stats* stats = impact_get_stats(m);
        if (stats) {{
            printf("Number of outer iterations: %d\\n", stats->n_sqp_iter);
            printf("Stop criterion: %d\\n", stats->sqp_stop_crit);
            printf("Runtime [s]: %e\\n", stats->runtime);
        }} else {{
            printf("No stats available.\\n");
        }}

        impact_print_problem(m);

        impact_hotstart(m);

        impact_print_problem(m);

        impact_solve(m);

        /* Allocate scratch space for state and control trajectories */
        impact_get_size(m, "u_opt", IMPACT_ALL, IMPACT_EVERYWHERE, IMPACT_FULL, &n_row, &n_col);
        printf("u_opt dims: %d - %d\\n", n_row, n_col);
        /* u_scratch = malloc(sizeof(double)*n_row*n_col); */
        u_scratch = (double *)malloc(sizeof(double)*n_row);

        impact_get_size(m, "x_opt", IMPACT_ALL, IMPACT_EVERYWHERE, IMPACT_FULL, &n_row, &n_col);
        printf("x_opt dims: %d - %d\\n", n_row, n_col);
        x_scratch = (double *)malloc(sizeof(double)*n_row*n_col);

        ROS_INFO("Solving single OCP");

        impact_get(m, "x_opt", IMPACT_ALL, IMPACT_EVERYWHERE, x_scratch, IMPACT_FULL);
        for (i=0;i<n_col;++i) {{
            for (j=0;j<n_row;++j) {{
                printf("%0.3e ", x_scratch[i*n_row+j]);
            }}
            printf("\\n");
        }}

        free(x_scratch);

        x_scratch = (double *)malloc(sizeof(double)*n_row);

        ROS_INFO("Starting MPC simulation");


        while (ros::ok()) {{
                ROS_INFO("Hello");

                impact_solve(m);
                impact_get(m, "u_opt", IMPACT_ALL, 0, u_scratch, IMPACT_FULL);
                impact_get(m, "x_opt", IMPACT_ALL, 1, x_scratch, IMPACT_FULL);
                impact_set(m, "x_current", IMPACT_ALL, 0, x_scratch, IMPACT_FULL);

                for (j=0;j<n_row;++j) {{
                    printf("%0.3e ", x_scratch[j]);
                }}
                printf("\\n");

                rate.sleep();
        }}

        free(u_scratch);
        free(x_scratch);

        impact_destroy(m);

}}

        """.format(**self.values_dict)

        return cpp_node_code

    def export_package(self, directory="ROS_package"):

        import os
        import shutil

        PKG_NAME = self.values_dict['PKG_NAME']
        ROS_PKG_ROOT = f"{directory}/{PKG_NAME}"
        IMPACT_NODE_NAME = self.values_dict['IMPACT_NODE_NAME']

        package_xml_code = self.generate_package_xml()
        cmakelists_code = self.generate_CMakeLists()
        cpp_node_code = self.generate_ros_node_cpp()
        # cpp_node_code = self.generate_hello_world_ros_node_cpp()

        if not os.path.exists(ROS_PKG_ROOT):
            os.makedirs(f"{ROS_PKG_ROOT}/src")
            os.makedirs(f"{ROS_PKG_ROOT}/include/{PKG_NAME}")

        # Create package.xml
        with open(f"{ROS_PKG_ROOT}/package.xml", mode='w') as f:
            f.write(package_xml_code)
        # Create CMakeLists.txt
        with open(f"{ROS_PKG_ROOT}/CMakeLists.txt", mode='w') as f:
            f.write(cmakelists_code)
        # Create ROS node from Impact mpc
        with open(f"{ROS_PKG_ROOT}/src/{IMPACT_NODE_NAME}.cpp", mode='w') as f:
            f.write(cpp_node_code)

        # Get files from build_dir
        build_dir_path = f"{PKG_NAME}_build_dir"
        if os.path.exists(build_dir_path):
            # Copy .c files to source directory
            shutil.copy(f"{build_dir_path}/{PKG_NAME}.c", f"{ROS_PKG_ROOT}/src/{PKG_NAME}.c")
            shutil.copy(f"{build_dir_path}/{PKG_NAME}_codegen.c", f"{ROS_PKG_ROOT}/src/{PKG_NAME}_codegen.c")
            # Copy header files to include directory
            shutil.copy(f"{build_dir_path}/{PKG_NAME}.h", f"{ROS_PKG_ROOT}/include/{PKG_NAME}/{PKG_NAME}.h")
            shutil.copy(f"{build_dir_path}/{PKG_NAME}_codegen.h", f"{ROS_PKG_ROOT}/include/{PKG_NAME}/{PKG_NAME}_codegen.h") 


            
            
            model_c = f"{build_dir_path}/{PKG_NAME}_model.c"
            model_h = f"{build_dir_path}/{PKG_NAME}_model.h"
            if os.path.exists(model_c):
                shutil.copy(model_c, f"{ROS_PKG_ROOT}/src/{PKG_NAME}_model.c")
            if os.path.exists(model_h):
                shutil.copy(model_h, f"{ROS_PKG_ROOT}/include/{PKG_NAME}/{PKG_NAME}_model.h")




# author = {'name': 'Arthur Impact'}
# maintainer = {'email': 'arthur@maintain.be'}

# ROSgen = ROSGenerator(name='ROSimpact', version='0.0.3', author=author, maintainer=maintainer)
# ROSgen.export_package()



