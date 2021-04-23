CMD="motor_stop"
PORT=/dev/ttyUSB0

function run()
{
	echo $CMD >$PORT
	sleep 1
}

function runleft()
{
	CMD="123 left forward motor"
	run
}

function runright()
{
	CMD="123 right forward motor"
	run
}

function stop()
{
	CMD="motor_stop"
	run

}

clear
inp=""
killall tail
sleep 1
tail -f $PORT &
while true
do
  read inp
  if [ "$inp" == "quit" ]
  then 
  	break
  fi
  echo $inp >$PORT
  
 done
 killall tail
 killall tail

