# 1. PI service settings
    /etc/systemd/system/pi_cam_imu.service

    ```
    [Unit]
    Description=Pi Camera+IMU TCP streamer
    After=network-online.target
    Wants=network-online.target

    [Service]
    Type=simple
    Environment=PYTHONUNBUFFERED=1
    ExecStart=/usr/bin/python3 /home/pi/code/Pi-senser-interface/pi_cam_stream.py
    Restart=always
    RestartSec=3
    # Make stdout/stderr go to the journal
    StandardOutput=journal
    StandardError=journal
    # Give it a sane working dir (for any relative paths)
    WorkingDirectory=/home/pi/code/Pi-senser-interface

    [Install]
    WantedBy=multi-user.target
    ```

# 2. service cheat sheet
after code edit: \
    ```
    sudo systemctl restart pi_cam_imu.service
    sudo systemctl status  pi_cam_imu.service --no-pager -l 
    journalctl -u pi_cam_imu.service -n 40 --no-pager
    ```

After you edit the service file (unit) \
    ```
    sudo systemctl daemon-reload
    sudo systemctl restart pi_cam_imu.service
    sudo systemctl status  pi_cam_imu.service --no-pager -l
    ```

enable at boot / disable \
    ```sudo systemctl enable  pi_cam_imu.service
    sudo systemctl disable pi_cam_imu.service```

Start/stop/quick logs \
    ```sudo systemctl start  pi_cam_imu.service
    sudo systemctl stop   pi_cam_imu.service
    journalctl -u pi_cam_imu.service -e -f    # live tail```

Edit with safety (keeps a drop-in override) \
    ```sudo systemctl edit --full pi_cam_imu.service
    # or just an override file:
    sudo systemctl edit pi_cam_imu.service
    # then:
    sudo systemctl daemon-reload
    sudo systemctl restart pi_cam_imu.service```

Verify it’s listening (video :6000, IMU :6001) \
    ```ss -lntp | grep -E ':6000|:6001'```

One-liner I use after pulling changes \
    ```cd ~/code/Pi-senser-interface \
    && git pull \
    && sudo systemctl restart pi_cam_imu.service \
    && journalctl -u pi_cam_imu.service -n 30 --no-pager```
    
Quick health checks \
    ```
    # recent restarts / failures
    systemctl status pi_cam_imu.service --no-pager -l
    journalctl -u pi_cam_imu.service --since "10 min ago"

    # power/thermal issues
    vcgencmd get_throttled
    vcgencmd measure_temp

    # last reboot times
    last -x | head -n 5
    ```