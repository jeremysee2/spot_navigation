## Deploying the Docker container on Spot CORE

Docker containers can be gotten from Docker Hub, at [jeremysee/spot_navigation:main](https://hub.docker.com/r/jeremysee/spot_navigation).

In order to deploy the Docker container, the following parameters need to be set:

* COMMAND overrides: `--host-ip 192.168.50.5 --guid <SPOT_USERNAME> --secret <SPOT_PASSWORD> 192.168.50.3`
* Enable Interactive mode, to allow terminal access after starting the container
* Networking: `host` mode to share the networking namespace with the Spot CORE
* Environmental variables: `SPOT_USERNAME, SPOT_PASSWORD, SPOT_REAL_DATA=true, USE_APRILTAG=true, DISPLAY=:0`
* Bind volume: host: `/home/spot/.Xauthority` to container: `/root/.Xauthority`, with write access

## GUI Applications through VNC on Spot CORE

This will allow GUI applications like RViz to be started from the Docker container and show in the Spot CORE. To view those applications, use [VNC](https://dev.bostondynamics.com/docs/payload/spot_core_vnc).

1. Start a SSH tunnel to Spot CORE for port 21000: `ssh -4 -p 20022 spot@192.168.80.3 -L 21000:127.0.0.1:21000`
2. Start the `systemctl` service for vncserver:
```bash
sudo systemctl enable vncserver@15100 # Enable service with the desired port.
sudo systemctl start vncserver@15100  # Start service.
systemctl status vncserver@15100 # Check the service is running.
```
3. Set a password with `vncpasswd`, usually 123456 if security isnt important when running without internet access
4. Use your VNC viewer on your laptop, TigerVNC or RealVNC etc, to connect to `localhost:21000`
5. Login with your VNC password
6. Login to the Ubuntu 18.04 desktop with your Spot CORE credentials
7. Start a interactive bash terminal to the existing Docker container with `docker exec -it <CONTAINER> /bin/bash`
