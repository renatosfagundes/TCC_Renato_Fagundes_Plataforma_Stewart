function [Ls] = desenhaPlataforma2(x,y,z,ax,ay,az,alfas,xs, ys, zs)
%     tic
    %% Definicao de constantes da plataforma de Stewart 
    L = .092563; %DistAncia do centro da base/plataforma ate cada um dos seis pontos do hexagono que a define
    H = 0.005; %DistAncia entre as faces inferior e superior da base/plataforma
    angs = deg2rad([39.3, 80.7, 159.3, -159.3, -80.7, -39.3])'; %Angulos entre eixo do eixo x do sistema de coordenadas e cada um dos seis pontos do hexagono 
    p(1:6,:) = [L*cos(angs),L*sin(angs),zeros(6,1),ones(6,1)]; %pontos do hexagono superior
    p(7:12,:) = [L*cos(angs),L*sin(angs),H*ones(6,1),ones(6,1)]; %pontos do hexagono inferior
    %Para usar a funcao fill3 para desenhar, e preciso indicar quais sao os
    %lados que deverao ser preenchidos. Abaixo sao definidos o topo, a
    %parte de baixo, e as 6 laterais do corpo que forma a plataforma. 
    topo = 1:6; %Quais sao os pontos de cima
    baixo = 7:12; %Quais sao os pontos de baixo
    lado(1,:) = [1, 2, 8, 7]; 
    lado(2,:) = [2, 3, 9, 8];
    lado(3,:) = [3, 4, 10, 9];
    lado(4,:) = [4, 5, 11, 10];
    lado(5,:) = [5, 6, 12, 11];
    lado(6,:) = [6, 1, 7, 12];
    
    angs = deg2rad([18.9 101.1 138.9 -138.9 -101.1 -18.9])'; %angulos dos pontos da plataforma em relacao ao scp
    LP = 0.091547; %distancia entre os pontos da plataforma e a origem do scp
    p2(1:6,:) = [LP*cos(angs),LP*sin(angs),zeros(6,1),ones(6,1)]; %pontos da plataforma em relacao a origem do SCP

    H0 = 0.175;
    %Pontos dos motores
    betas = deg2rad([34.3 85.7 154.3 -154.3 -85.7 -34.3])'; %angulos dos motores em relacao ao centro da base
    DS = 0.086322; %distancia entre eixo do servo e origem da base 
    b(1:6,:) = [DS*cos(betas),DS*sin(betas),H/2*ones(6,1),ones(6,1)]; %posicao dos eixos dos servos em relacao a origem 
    
    ac = [0 2*pi/3 -2*pi/3];
    c = b+0.01*[cos(ac(1)) sin(ac(1)) 0 1; cos(ac(2)) sin(ac(2)) 0 1; ...
        cos(ac(2)) sin(ac(2)) 0 1; cos(ac(3)) sin(ac(3)) 0 1; ...
        cos(ac(3)) sin(ac(3)) 0 1; cos(ac(1)) sin(ac(1)) 0 1];
    
    a = 0.047753; %comprimento do braa§o do servo em metros

    hold off

    %Pontos da plataforma
    RBT = Tr(x,y,z+H0)*Rotz(az)*Roty(ay)*Rotx(ax); %Rotacao desejada ''
    p1 = RBT*Rotz(deg2rad(60))*p';
    p1 = p1';


    % d = c(:,1:3)+a*[sin(ac(1)) cos(ac(1)) 0; sin(ac(2)) -cos(ac(2)) 0; ...
    %     -sin(ac(2)) cos(ac(2)) 0; sin(ac(3)) -cos(ac(3)) 0; ...
    %     -sin(ac(3)) cos(ac(3)) 0; sin(ac(1)) -cos(ac(1)) 0];
    d = a*[zeros(6,1) ones(6,1) zeros(6,1)];
    d = [d ones(6,1)];
    angsdz = [ac(1) ac(2)+pi ac(2) ac(3)+pi ac(3) ac(1)+pi];

    for cont = 1:6
       d(cont,:) = Tr(c(cont,1),c(cont,2),c(cont,3))*Rotz(angsdz(cont))*Rotx(alfas(cont))*d(cont,:)';
    end
    % plot3(d(:,1),d(:,2),d(:,3),'.','Color','y','MarkerSize',20);

    q = RBT*p2'; %pontos da plataforma em relacao a origem do SCb
    q = q(1:3,:)';
    plot3(q(:,1),q(:,2),q(:,3),'.','Color','k','MarkerSize',20);
    fill3(p(topo,1),p(topo,2),p(topo,3),'r');
    hold on
    fill3(p(baixo,1),p(baixo,2),p(baixo,3),'r');

    fill3(p1(topo,1),p1(topo,2),p1(topo,3),'r');
    fill3(p1(baixo,1),p1(baixo,2),p1(baixo,3),'r');
    
    d_ = 0.04;
    axis([-(L+d_) L+d_ -(L+d_) (L+d_) -.05 .25]);
    
    for cont = 1:6
        fill3(p1(lado(cont,:),1),p1(lado(cont,:),2),p1(lado(cont,:),3),'r');
        fill3(p(lado(cont,:),1),p(lado(cont,:),2),p(lado(cont,:),3),'r');
        ex = [b(cont,1); c(cont,1); d(cont,1); q(cont,1)];
        ey = [b(cont,2); c(cont,2); d(cont,2); q(cont,2)];
        ez = [b(cont,3); c(cont,3); d(cont,3); q(cont,3)];
        
        plot3(ex,ey,ez,'.-','Color','k','MarkerSize',20,'LineWidth',4);
        plot3([c(cont,1) q(cont,1)],[c(cont,2) q(cont,2)],[c(cont,3) q(cont,3)],'--','Color','b','MarkerSize',15,'LineWidth',2);
        plot3(xs,ys,zs+H0+0.025,'Color','k','LineWidth',2);
    end
    drawnow
%     box on
%     grid on
%     xlabel('x (m)')
%     ylabel('y (m)')
%     zlabel('z (m)')
    Ls = sqrt(sum((q(:,1:3)-d(:,1:3)).^2,2));