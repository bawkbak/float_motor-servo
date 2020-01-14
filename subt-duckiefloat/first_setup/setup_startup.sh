#!/bin/sh
echo -e "[Unit]\n\
Description=/etc/rc.local Compatibility\n\
ConditionPathExists=/etc/rc.local\n\
\n\
[Service]\n\
Type=forking\n\
ExecStart=/etc/rc.local start\n\
TimeoutSec=0\n\
StandardOutput=tty\n\
RemainAfterExit=yes\n\
SysVStartPriority=99\n\
\n\
[Install]\n\
WantedBy=multi-user.target\n" | sudo tee /etc/systemd/system/rc-local.service

echo -e "#!/bin/sh -e\n\
# rc.local\n\
#\n\
# This script is executed at the end of each multiuser runlevel.\n\
# Make sure that the script will "exit 0" on success or any other\n\
# value on error.\n\
#\n\
# In order to enable or disable this script just change the execution\n\
# bits.\n\
#\n\
# By default this script does nothing.\n\
\n\
echo \"good\" > /usr/local/test.log\n\
\n\
exit 0" | sudo tee /etc/rc.local

sudo chmod +x /etc/rc.local
sudo systemctl enable rc-local
sudo systemctl start rc-local.service
sudo systemctl status rc-local.service

