SEQ=$1
# gdb -ex run --args
./build/bin/sdso_dataset \
    files=$SEQ \
    calib=$SEQ/stereo_camera.txt \
    mode=1 \
    nogui=0 \
    quiet=1 \
    start=500 \
    end=10000