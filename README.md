This project is forked from Colosseum Near-Real-Time RIC (https://github.com/wineslab/colosseum-near-rt-ric/tree/ns-o-ran)

git clone -b ns-o-ran https://github.com/wineslab/colosseum-near-rt-ric
cd colosseum-near-rt-ric/setup-scripts

./import-wines-images.sh  # import and tag
./setup-ric-bronze.sh  # setup and launch

# Terminal 1 logs the E2Term
docker logs e2term -f --since=1s 2>&1 | grep gnb:  # this will help to show only when a gnb is interacting

# Terminal 2 builds and run the x-app container
cd colosseum-near-rt-ric/setup-scripts
./start-xapp-ns-o-ran.sh

cd /home/sample-xapp
./run_xapp.sh
