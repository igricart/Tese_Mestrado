function [saida]= sinalruido(u)
global buffersup bufferinf 
%  

buffersup = [buffersup(2:end,1); u]
sup = max(buffersup)

bufferinf = [bufferinf(2:end,1); u]
inf = min(bufferinf)
%

sinal = (mean([sup inf]));
ruido = (sup-inf)-sinal;

% O ruido nao pode ser mais da metade do sinal --> N<S/2 --> N/S<1/2=maxNSR
% 
maxNSR=1/2;
if (sinal==0)
    if (ruido==0) 
        NSR=0;
    else
        NSR=maxNSR;
    end
else
    NSR=ruido/sinal;
    if (NSR>1/2) NSR=1/2;
    end
end

%
saida=[NSR sinal sup inf ruido u];
%

end
