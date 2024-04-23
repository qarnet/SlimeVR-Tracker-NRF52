from os.path import join, isfile, os, sys

Import("env")

pin_files = ["variant.h", 
             "variant.cpp"]

# bootloader_files = ["update-feather_nrf52840_express_bootloader-0.8.0_nosd.uf2",
#                     "feather_nrf52840_express_bootloader-0.8.0_s140_6.1.1.zip",
#                     "feather_nrf52840_express_bootloader-0.8.0_s140_6.1.1.hex"]

FRAMEWORK_DIR = env.PioPlatform().get_package_dir("framework-arduinoadafruitnrf52")

IS_WINDOWS = sys.platform.startswith("win")

if not IS_WINDOWS:
    patch_path = "patch/"
    pin_files_path = FRAMEWORK_DIR + "/variants/feather_nrf52840_express/"
    # bootloader_files_path = FRAMEWORK_DIR + "/bootloader/feather_nrf52840_express/"

    for file in pin_files:
        env.Execute("cp -u " + patch_path + file + " " + pin_files_path + file)
    # for file in bootloader_files:
    #     env.Execute("cp -u " + patch_path + file + " " + bootloader_files_path + file)