* `sudo apt-get update`
* `sudo apt-get install -y nvidia-container-toolkit`
* `sudo systemctl restart docker`

* `docker run --gpus all -it  -v "$(pwd):/workspace" -p 7007:7007 ghcr.io/nerfstudio-project/nerfstudio:latest`

#### If you have a video
* `ns-process-data video --data /workspace/your_video.mp4 --output-dir /workspace/processed/`
* `ns-train splatfacto --data /workspace/processed/`

**Note:** If gpus are note detected, exit and `sudo systemctl restart docker`
#### Easy way to test by downloading a working dataset
* `ns-download-data nerfstudio --capture-name=poster`
* `ns-train splatfacto --data data/nerfstudio/poster`

During the training, the progress can be viewed at `http://localhost:7007/`


#### Export the splat data
* `ns-export gaussian-splat --load-config outputs/processed/splatfacto/TIMESTAMP/config.yml --output-dir exports/`

#### Error
Sometime the following error happens
`Intel MKL FATAL ERROR: Cannot load /usr/local/lib/python3.10/dist-packages/open3d/cpu/pybind.cpython-310-x86_64-linux-gnu.so.`

So, inside the docker,
* `sudo apt update`
* `sudo apt install python3-pip`
* `pip uninstall open3d`
* `pip install open3d==0.16.0`

Run the export again to get a `splat.py` inside exports folder. This can be viewed by dragging and dropping into [superspl](https://superspl.at/editor)