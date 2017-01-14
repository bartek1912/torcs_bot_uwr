#/bin/bash

echo $1 "parametrow"

for i in `seq 1 $1`
do
	./client port:300$i &
done

