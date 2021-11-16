function alphas = calculaAngulos2(x,y,z,ax,ay,az)
%     tic
    a = 0.047753; %comprimento do braço do servo em metros
    s = 0.185; %comprimento da haste em metros

    H0 = 0.175; %altura inicial da plataforma em metros

    betas = [0.5391    1.5553    2.6335   -2.6335   -1.5553   -0.5391]'; %angulos dos motores em relação ao centro da base
    DS = 0.0948; %distancia entre eixo do servo e origem da base 
    b(1:6,:) = [DS*cos(betas),DS*sin(betas),0.0025*ones(6,1),ones(6,1)]; %posicao dos eixos dos servos em relacao a origem 

    angs = deg2rad([18.9 101.1 138.9 -138.9 -101.1 -18.9])'; %angulos dos pontos da plataforma em relacao ao scp (sistema de coordenadas da plataforma)
    LP = 0.091547; %distancia entre os pontos da plataforma e a origem do scp
    p(1:6,:) = [LP*cos(angs),LP*sin(angs),zeros(6,1),ones(6,1)]; %pontos da plataforma em relacao a origem do SCP

    T = Tr(x,y,z+H0); %translacao desejada da origem do SCP em relacao ao SCB (sistema de coordenadas da base)
    RB = Rotz(az)*Roty(ay)*Rotx(ax); %Rotacao desejada ''
    q = T*RB*p'; %pontos da plataforma em relacao a origem do SCb
    q = q';    
    
    betas = deg2rad([0 300 120 -300 -120 180]+90)';
    
    for k = 1:6
        l = (q(k,1:3) - b(k,1:3));
        absl = sqrt(sum(l.^2));
        ek = 2*a*l(3);
        fk = 2*a*(cos(betas(k)).*l(1)+sin(betas(k)).*l(2));
        gk = absl^2-(s^2-a^2);

        alphas(k) = (asin(gk/sqrt(ek^2+fk^2))-atan(fk/ek));
    end
%     toc
end