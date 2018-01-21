#!/bin/sh

mkdir data 
cd data
wget https://www.dropbox.com/s/yz1pi6l2vs0umf2/wasp_registration.tar.gz?dl=0 -O wasp_registration.tar.gz
tar -xvf wasp_registration.tar.gz
rm wasp_registration.tar.gz
