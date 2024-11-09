#!/bin/sh
# Script to import the base images to create the RIC containers from Wineslab Docker Hub

sudo docker start e2term
sudo docker start e2mgr
sudo docker start e2rtmansim
sudo docker start db

sudo docker start sample-xapp-24
