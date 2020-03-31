all:
	gcc -Wall *.c -o AZTEK_run  -lm

clean:
	rm -rfv AZTEK_run
