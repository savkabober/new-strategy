all: 
	g++ main.cpp -o app && ./app
cuda:
	nvcc ./cuda_directory/main.cu -o cuda_app && ./cuda_app
clean:
	rm -f ./app ./cuda_app