function saida=supmoving(u)
global buffer2


%  

buffer2=wshift('1D',buffer2,1);
buffer2(end)=abs(u);
saida=max(buffer2);


end
