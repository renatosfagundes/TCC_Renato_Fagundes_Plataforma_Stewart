clear; clc; close;

t = 0:.1:3*pi; %vetor de tempo a partir do qual sao definidas as equacoes parametricas
xs = 0.01*cos(t); %deslocamento no eixo x
ys = 0.01*sin(t); %deslocamento no eixo y
zs = linspace(0.01,-0.01,length(t)); %deslocamento no eixo z
axs = 0*(pi/8).*sin(t-pi).*(t>pi).*(t<3*pi); %angulo roll
ays = 0*(pi/8).*sin(t-4*pi).*(t>4*pi).*(t<6*pi); %angulo pitch
azs = 0*(pi/8).*sin(t-7*pi).*(t>7*pi).*(t<9*pi); %angulo yaw

for cont = 1:length(xs)
    alphas = calculaAngulos2(xs(cont),ys(cont),zs(cont),axs(cont),ays(cont),azs(cont)); %calculo dos angulos
    desenhaPlataforma2(xs(cont),ys(cont),zs(cont),axs(cont),ays(cont),azs(cont),alphas,xs(1:cont),ys(1:cont),zs(1:cont)); %desenho da plataforma com saida suprimida
    pause(0.01);
end