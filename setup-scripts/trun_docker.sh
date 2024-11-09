#!/bin/sh

RIC_SUBNET=10.0.2.0/24
RIC_IP=10.0.2.1
E2TERM_IP=10.0.2.10
E2TERM_SCTP_PORT=36422
E2MGR_IP=10.0.2.11
DBAAS_IP=10.0.2.12
DBAAS_PORT=6379
E2RTMANSIM_IP=10.0.2.15
XAPP_IP=10.0.2.24  # generic xApp IP

# Create a route info file to tell the containers where to send various
# messages. This will be mounted on the containers
ROUTERFILE=`pwd`/router.txt
cat << EOF > $ROUTERFILE
newrt|start
rte|10020|$E2MGR_IP:3801
rte|10060|$E2TERM_IP:38000
rte|10061|$E2MGR_IP:3801
rte|10062|$E2MGR_IP:3801
rte|10070|$E2MGR_IP:3801
rte|10071|$E2MGR_IP:3801
rte|10080|$E2MGR_IP:3801
rte|10081|$E2TERM_IP:38000
rte|10082|$E2TERM_IP:38000
rte|10360|$E2TERM_IP:38000
rte|10361|$E2MGR_IP:3801
rte|10362|$E2MGR_IP:3801
rte|10370|$E2MGR_IP:3801
rte|10371|$E2TERM_IP:38000
rte|10372|$E2TERM_IP:38000
rte|1080|$E2MGR_IP:3801
rte|1090|$E2TERM_IP:38000
rte|1100|$E2MGR_IP:3801
rte|12010|$E2MGR_IP:38010
rte|1101|$E2TERM_IP:38000
rte|12002|$E2TERM_IP:38000
rte|12003|$E2TERM_IP:38000
rte|10091|$E2MGR_IP:4801
rte|10092|$E2MGR_IP:4801
rte|1101|$E2TERM_IP:38000
rte|1102|$E2MGR_IP:3801
rte|12001|$E2MGR_IP:3801
mse|12050|$(echo $XAPP_IP | cut -d "." -f 4)|$XAPP_IP:4560
newrt|end
EOF


$SUDO docker run -d --network ric --ip $DBAAS_IP --name db dbaas:bronze
$SUDO docker run -d -it --network ric --ip $E2RTMANSIM_IP --name e2rtmansim e2rtmansim:bronze
$SUDO docker run -d -it --network ric --ip $E2MGR_IP -e RIC_ID=7b0000-000000/18 \
    -e DBAAS_PORT_6379_TCP_ADDR=$DBAAS_IP -e DBAAS_PORT_6379_TCP_PORT="6379" \
    -e DBAAS_SERVICE_HOST=$DBAAS_IP -e DBAAS_SERCE_PORT="6379" \
    --mount type=bind,source=$ROUTERFILE,destination=/opt/E2Manager/router.txt,ro \
    --name e2mgr e2mgr:bronze
$SUDO docker run -d -it --network=ric --ip $E2TERM_IP --name e2term \
        --mount type=bind,source=$ROUTERFILE,destination=/opt/e2/dockerRouter.txt,ro \
        $E2TERM_CONFIG_BIND \
        e2term:bronze
        
ENTRYPOINT=/bin/bash
XAPP_IP=$XAPP_IP
XAPP_ID=$(echo $XAPP_IP | cut -d "." -f 4)
CONTAINER_NAME=${IMAGE_NAME}-${XAPP_ID}

$SUDO docker run -d -it --entrypoint ${ENTRYPOINT} --network ric --ip ${XAPP_IP} \
    -e DBAAS_SERVICE_HOST=$DBAAS_IP -e DBAAS_SERVICE_PORT=$DBAAS_PORT --name ${CONTAINER_NAME} ${IMAGE_NAME}:latest

