#!/bin/bash
# intern taught tme how to make this 
scp supervisor/supervisord.conf pi@10.1.0.31:/tmp/.
ssh pi@10.1.0.31 'sudo mv /tmp/supervisord.conf /etc/supervisor/.'
ssh pi@10.1.0.31 sudo systemctl restart supervisor