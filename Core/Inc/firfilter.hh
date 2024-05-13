#pragma once
#include <cstdint>

namespace FIR{

template <size_t N>
class Filter{
	private:
	float const * const impulseResponse;
	float* ringbuf{nullptr};
	size_t ringbuf_index{0};
	float out{0.0f};
public:
	Filter(float const * const impulseResponseOfLenght_N):impulseResponse(impulseResponseOfLenght_N){
		ringbuf = new float[N];
		for(size_t i=0;i<N;i++) ringbuf[i]=0.0f;

	}
	~Filter(){
		delete ringbuf;
	}

	float Update(float f){
		ringbuf[ringbuf_index]=f;

		ringbuf_index++;

		if(ringbuf_index==N){
			ringbuf_index=0;
		}

		float temp=0;
		size_t sum_index=ringbuf_index;

		for(size_t n=0; n<N;n++){
			if(sum_index>0){
				sum_index--;
			}else{
				sum_index=N-1;
			}
			//Die folgende Zeile müssen die Studis selbst hinbekommen
			temp+=impulseResponse[n]*ringbuf[sum_index];
		}
		out=temp;
		return temp;
	}

	float GetOut(){
		return out;
	}

};

}
