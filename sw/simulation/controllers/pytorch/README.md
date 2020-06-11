#pytorch

Example controller to run pytorch within Swarmulator.

## Instructions
1. Download the C++ version of libtorch as per https://pytorch.org/cppdocs/installing.html

Navigate to a directory of choice and download:
```wget https://download.pytorch.org/libtorch/nightly/cpu/libtorch-shared-with-deps-latest.zip```
```unzip libtorch-shared-with-deps-latest.zip```

2. Set an environment variable ```TORCH_LIB_HOME```
that tells swamulator the absolute path of where you placed your pytorch library.

``` export TORCH_LIB_HOME=/absolute/path/to/torchlib```

2. Add ```TORCH_LIB_HOME``` to ```LD_LIBRARY_PATH```

``` export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:$TORCH_LIB_HOME/lib```

3. Just as in the example on https://pytorch.org/cppdocs/installing.html, you can now include ```<torch/torch.h>``` in your controller.

4. Go nuts!

## Troubleshooting and known prior issues
- I get a compile error due to ```google/protobuf/...```. This is due to caffe2, which you may not need (and if you get this error you likely don't). For the purposes of swarmulator, you can hack a quick fix by deleting the ```caffe2``` folder in ```libtorch/include```. Or you can install the necessary packages :).

- ```<torch/torch.h>``` not found. If this happens, check that TORCH_LIB_HOME is correctly defined as an environment variable. (pro tip: add it to bashrc)

- Runtime error: ```./swarmulator: error while loading shared libraries: libc10.so: cannot open shared object file: No such file or directory```.
Check that TORCH_LIB_HOME/lib is in LD_LIBRARY_PATH!
