#!/bin/bash

# Change to script directory
cd "$(dirname "$0")" || exit 1

echo "Copying files..."
cp -v "../../tone.cfg" \
      "../../cfg_tool.bin" \
      "../../app.bin" \
      "../../bd19loader.bin" \
      "../../p11_code.bin" \
      "../../script.ver" \
      "../../flash_params.bin" .

echo "Running isd_download..."
"../../isd_download" "../../isd_config.ini" -tonorflash -dev bd19 -boot 0x2000 -div8 -wait 300 -uboot "../../uboot.boot" -app "../../app.bin" "../../cfg_tool.bin" -res "../../p11_code.bin" -uboot_compress -flash-params flash_params.bin || exit 1

echo "Cleaning up..."
rm -f *.mp3 *.PIX *.TAB *.res *.sty

echo "Generating upgrade file..."
"../../fw_add" -noenc -fw jl_isd.fw -add "../../ota.bin" -type 100 -out jl_isd.fw
"../../fw_add" -noenc -fw jl_isd.fw -add script.ver -out jl_isd.fw

echo "Creating UFW file..."
"../../ufw_maker" -fw_to_ufw jl_isd.fw
cp -v jl_isd.ufw update.ufw
rm -f jl_isd.ufw

echo "Waiting 2 seconds..."
sleep 2

echo "Done!"