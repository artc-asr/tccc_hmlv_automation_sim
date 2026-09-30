# gpu_nvidia.sh  --  route Gazebo (ogre2) rendering to the NVIDIA dGPU.
#
# WHY: on a hybrid Intel-iGPU + NVIDIA-dGPU laptop (PRIME "on-demand"), GL/EGL
# goes to the Intel iGPU by default. On a very new iGPU whose Mesa doesn't know
# the PCI id (e.g. Meteor Lake 0x7d67) gz's ogre2 dies with "failed to create
# dri2 screen" / MESA errors. The NVIDIA card renders ogre2 fine; these vars
# force everything onto it. Harmless no-op on a desktop NVIDIA box.
#
# Covers BOTH render paths:
#   * GLX  -> the gz 3D GUI window            (__GLX_VENDOR_LIBRARY_NAME)
#   * EGL  -> the headless sensor rendering    (__EGL_VENDOR_LIBRARY_FILENAMES)
#            i.e. gpu_lidar + cameras -- what navigation/SLAM needs.
#
# USAGE (this shell + everything it launches):
#   source gpu_nvidia.sh            # or: source <robot>/source_sim.sh gpu, ./sim.sh <robot> gpu
#   ros2 launch ...                 # gz GUI + gpu sensors now render on NVIDIA
#
# Only affects the current shell; open a normal shell to go back to the iGPU.
# Not sourced automatically -- it's laptop-specific, opt-in via the `gpu` flag.

export __NV_PRIME_RENDER_OFFLOAD=1
export __GLX_VENDOR_LIBRARY_NAME=nvidia
export __VK_LAYER_NV_optimus=NVIDIA_only
# expose ONLY the NVIDIA EGL vendor so ogre2's headless EGL device == NVIDIA
# (otherwise it enumerates the Intel EGL device first and fails).
export __EGL_VENDOR_LIBRARY_FILENAMES=/usr/share/glvnd/egl_vendor.d/10_nvidia.json

echo "[gpu_nvidia] Gazebo will render on the NVIDIA dGPU (PRIME offload)."
echo "[gpu_nvidia] verify: glxinfo -B | grep 'OpenGL renderer'  -> should say NVIDIA"
