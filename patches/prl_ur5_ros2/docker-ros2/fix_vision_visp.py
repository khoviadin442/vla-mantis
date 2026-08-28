#!/usr/bin/env python3
"""Patch upstream bugs in lagadic/vision_visp (rolling) that stop visp_auto_tracker running.

Run from the directory holding the freshly cloned `vision_visp`. Each fix asserts that its
anchor matches exactly once, so an upstream change fails the build loudly instead of silently
producing a broken tracker.

1. visp_tracker/CMakeLists.txt unconditionally downloads a tutorial rosbag whose GitHub
   release asset returns 404, which fails the whole colcon build.
2. visp_auto_tracker builds libvisp_auto_tracker_common.so but its install(TARGETS ...) lists
   only the executable, so the node dies with "cannot open shared object file".
3. CmdLine::init() calls common() with uninitialised argc_/argv_; getopt_long walks that
   garbage and segfaults.
4. CmdLine::init() assigns config_file *before* common(), which resets it to the default
   models/config.cfg - so the caller's config, and the detector-type it selects, is ignored
   and the tracker always tries the zbar QR detector.
5. AutoTracker built the config path as model_path + model_name (no separator) and then
   prefixed model_path a second time.
"""
import pathlib
import sys

ROOT = pathlib.Path("vision_visp")


def patch(relative_path, old, new, label):
    p = ROOT / relative_path
    s = p.read_text()
    n = s.count(old)
    assert n == 1, f"{label}: anchor matched {n}x in {p} (upstream changed?)"
    p.write_text(s.replace(old, new))
    print(f"  patched {label}")


patch(
    "visp_tracker/CMakeLists.txt",
    """include(ExternalProject)
ExternalProject_Add(
    external_bag
    PREFIX "externals"
    URL https://github.com/lagadic/vision_visp/releases/download/vision_visp-0.5.0/tutorial-static-box-ros2.bag
    DOWNLOAD_NO_EXTRACT true
    CONFIGURE_COMMAND ""
    BUILD_COMMAND ""
    PATCH_COMMAND ""
    INSTALL_COMMAND ""
)

ExternalProject_Get_Property(external_bag DOWNLOADED_FILE)
if(EXISTS ${DOWNLOADED_FILE})
  message("Successfully download ${DOWNLOADED_FILE}")
endif()

install(DIRECTORY
  ${CMAKE_CURRENT_BINARY_DIR}/externals/src/tutorial-static-box-ros2.bag
  DESTINATION share/bag
)
message("Bagfile installed in ${CMAKE_INSTALL_PREFIX}/share/bag/tutorial-static-box-ros2")
""",
    "",
    "visp_tracker: drop the 404 tutorial-bag download",
)

patch(
    "visp_auto_tracker/CMakeLists.txt",
    "ament_package()",
    "install(TARGETS visp_auto_tracker_common LIBRARY DESTINATION lib)\nament_package()",
    "visp_auto_tracker: install the common library",
)

patch(
    "visp_auto_tracker/src/cmd_line.cpp",
    """void CmdLine::init(std::string &config_file)
{
  this->config_file = config_file;
  common();
  loadConfig(this->config_file);
}""",
    """void CmdLine::init(std::string &config_file)
{
  static char prog_name[] = "visp_auto_tracker";
  static char *dummy_argv[] = {prog_name, nullptr};
  argc_ = 1;
  argv_ = dummy_argv;
  common();
  this->config_file = config_file;
  loadConfig(this->config_file);
}""",
    "CmdLine::init: valid argv for getopt, and keep the caller's config file",
)

patch(
    "visp_auto_tracker/src/autotracker.cpp",
    """  model_full_path = model_path_ + model_name_;
  tracker_config_path_ = model_path_ + "/" + model_full_path + ".cfg";""",
    """  model_full_path = model_path_ + "/" + model_name_;
  tracker_config_path_ = model_full_path + ".cfg";""",
    "AutoTracker: build the config path correctly",
)

print("vision_visp patched")
sys.exit(0)
