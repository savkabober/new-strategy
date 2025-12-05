all: 
	g++ main.cpp -o app && ./app
clean:
	rm -f ./app