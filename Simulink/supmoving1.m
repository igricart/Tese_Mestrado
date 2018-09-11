function saida=supmoving1(u)
global bufferNSR


%  

bufferNSR=wshift('1D',bufferNSR,1);
bufferNSR(end)=abs(u);
saida=max(bufferNSR);


end
