function AnimacionCuatrupedo(y, dir)
    
    % Esta función recibe y = gamma, que es el angulo al cual esta la
    % primer pata del cuatrupedo. Asimismo, recibe 'dir' que se refiere a
    % la dirección a la que se quiere que vaya el robot. 
    %
    %---> 'dir' puede ser igual a: 'l', 'r', 'f', 'b', 'cw' y 'cc'.
    %
    % Ejemplo de corrida: AnimacionCuatrupedo(pi/4, 'l')
    
    %Se declaran los tiempo inicial y final, así como la delta de tiempo
    t0=0;   tf=2;  delta_t=0.1;
    
    %Se declaran las posiciones iniciales y finales para los dos pares de
    %patas. Recordemos que dos patas deben estar en el suelo mientras otras
    %dos en el aire, por eso estan desfasadas unas con las otras por 'pi'.
    a0=pi/2;  af=(5*pi)/2;  a02=(3*pi)/2;  af2=(7*pi)/2;
    
    %Switch case que se encarga de determinar la beta que es necesaria para
    %cumplir con la dirección a la que se quiere ir con el robot.
    switch dir
        % Left
        case 'l' 
            beta = pi/2;
            beta2 = pi/2;
        % Right
        case 'r' 
            beta = 3*pi/2;
            beta2 = 3*pi/2;
        % Backward
        case 'b' 
            beta = pi;
            beta2 = pi;
        % Forward
        case 'f' 
            beta = 0;
            beta2 = 0;
        % Counter Clockwise
        case 'cc' 
            beta = pi;
            beta2 = 0;
        % Clockwise
        case 'cw' 
            beta2 = pi;
            beta = 0;
        otherwise
            beta = 0;
            beta2 = 0;
    end
    
    % For para hacer una vez la trayectoria circular propuesta 
    % En cada iteración se del for se obtiene un punto de esta trayectoria.
    for t=t0:delta_t:tf
        
        %Se obtiene la w que ayuda a indicar en que "porcentaje" de la
        %trayectoria se va
        w=-(t-t0)/(tf-t0);
        
        %Se calculan alfa y alfa2 las cuales son clave para calcular los
        %puntos de la trayectoria. Se tienen dos debido al desfase de las
        %patas para tener unas en el suelo y otras en el aire
        alfa =(af  - a0) * w+a0;
        alfa2=(af2 - a02)* w+a02;
        
        %Se obtiene el punto exacto dentro de la trayectoria que
        %corresponde a la iteración en la que se va.
        %Pdi corresponde a la para i
        Pd  = LugarDeseado(alfa,  beta2, y         );
        Pd2 = LugarDeseado(alfa2, beta,  y+pi/2    );
        Pd3 = LugarDeseado(alfa,  beta,  y+pi      );
        Pd4 = LugarDeseado(alfa2, beta2, y+(3*pi/2));
        
        %Se obtienen los grados que se mandaran a las patas
        % Thetai corresponde a la pata i
        Theta  = IK_pata( Pd  );
        Theta2 = IK_pata( Pd2 );
        Theta3 = IK_pata( Pd3 );
        Theta4 = IK_pata( Pd4 );
        
        %Se mandan los valores obtenidos de la IK a la función que se
        %encargar de dibujar la pata
        plot_lapata(Theta, Theta2, Theta3, Theta4, y);
        
        %Se manda a llamar al igual, una función que dibuja la trayectoria
        %de cada pata en donde le toca. 
        plot_Trayectoria(beta2,y);
    end
end

function plot_lapata(Theta, Theta2, Theta3, Theta4, y)
    %Recibe 4 Theta's y la gamma=y
    %
    %Esta función se encarga de dibujar al cuatrupedo utilizando las
    %matrices de transoformación previamente calculadas usando la FK

    %Los valores de los eslabones del robot, en este caso se usaron los de
    %robot del proyecto
    L1=6.325;   L2=10.05;   L3=6.1;
    
    %Se generan las matrices de transformación de cada pata.
    %Se decidio hacerlo de esta forma tan manual para un mejor
    %entendimiento del programa. 
    % 
    %Ti corresponde a la pata i
    %
    %Se Ti{1} es la matriz que se utiliza para poder referenciar a la pata
    %desde el eje coordenado Universal. Previamente solo se tenian las 
    %Ti{2..5}
    T{1}=[rotz(y), rotz(y)*[3;0;0]; 0 0 0 1];
    %Ti{1} tiene una rotación de y=gamma, ya que el eje coordenado 0 de la
    %pata no puede estar en la misma dirección que el U, ya que cada pata
    %debe estar viendo en direccion distintas. 
    T{2}=MaTransHomoDH(Theta(1),   0,     0,    0);
    T{3}=MaTransHomoDH(Theta(2),   0,  pi/2,   L1);
    T{4}=MaTransHomoDH(Theta(3),   0,    pi,   L2);
    T{5}=[eye(3),[0;L3;0]; 0 0 0 1];
    
    T2{1}=[rotz(y+pi/2), rotz(y+pi/2)*[3;0;0]; 0 0 0 1];
    T2{2}=MaTransHomoDH(Theta2(1),   0,     0,    0);
    T2{3}=MaTransHomoDH(Theta2(2),   0,  pi/2,   L1);
    T2{4}=MaTransHomoDH(Theta2(3),   0,    pi,   L2);
    T2{5}=[eye(3),[0;L3;0]; 0 0 0 1];
    
    T3{1}=[rotz(y+pi), rotz(y+pi)*[3;0;0]; 0 0 0 1];
    T3{2}=MaTransHomoDH(Theta3(1),   0,     0,    0);
    T3{3}=MaTransHomoDH(Theta3(2),   0,  pi/2,   L1);
    T3{4}=MaTransHomoDH(Theta3(3),   0,    pi,   L2);
    T3{5}=[eye(3),[0;L3;0]; 0 0 0 1];
    
    T4{1}=[rotz(y+(3*pi/2)), rotz(y+(3*pi/2))*[3;0;0]; 0 0 0 1];
    T4{2}=MaTransHomoDH(Theta4(1),   0,     0,    0);
    T4{3}=MaTransHomoDH(Theta4(2),   0,  pi/2,   L1);
    T4{4}=MaTransHomoDH(Theta4(3),   0,    pi,   L2);
    T4{5}=[eye(3),[0;L3;0]; 0 0 0 1];
  
    
    %Se dibuja el eje universal 
    trplot(eye(4),'frame','0','color','k');
    axis([-20 20 -20 20 -20 20]);
    hold on
    
    O=[0;0;0;1]; T0i=eye(4); T1i=eye(4); T2i=eye(4); T3i=eye(4); 
    P=cell(1,5); P2=cell(1,5); P3=cell(1,5); P4=cell(1,5);
    
    % Esto genera los puntos y los sistemas coordenados de cada pata en
    % cada vertice
    for i=1:5
        % Se calculan los ejes coordenados de cada para por cada vertice y
        % se dibujan
        T0i=T0i*T{i};
        T1i=T1i*T2{i};
        T2i=T2i*T3{i};
        T3i=T3i*T4{i};
        
        trplot(T0i,'frame',num2str(i),'color','r');
        hold on
        trplot(T1i,'frame',num2str(i),'color','r');
        hold on
        trplot(T2i,'frame',num2str(i),'color','r');
        hold on
        trplot(T3i,'frame',num2str(i),'color','r');
        hold on
        
        %Se calculan los puntos para despues dibujar las lineas entre los
        %ejes
        P{i}=T0i*O;
        P2{i}=T1i*O;
        P3{i}=T2i*O;
        P4{i}=T3i*O;
    end
    
    %lineas que correspeonden al robot
    Xp=[O,[P{1:5}]]; Xp2=[O,[P2{1:5}]]; Xp3=[O,[P3{1:5}]]; Xp4=[O,[P4{1:5}]];
    plot3(Xp(1,:),Xp(2,:),Xp(3,:),'b*', ... 
          Xp(1,:),Xp(2,:),Xp(3,:),'b-');
    hold on
    plot3(Xp2(1,:),Xp2(2,:),Xp2(3,:),'b*', ... 
          Xp2(1,:),Xp2(2,:),Xp2(3,:),'b-');
    hold on
    plot3(Xp3(1,:),Xp3(2,:),Xp3(3,:),'b*', ... 
          Xp3(1,:),Xp3(2,:),Xp3(3,:),'b-');
    hold on
    plot3(Xp4(1,:),Xp4(2,:),Xp4(3,:),'b*', ... 
          Xp4(1,:),Xp4(2,:),Xp4(3,:),'b-');
end

function plot_Trayectoria(b,y)
    %Recibe beta=b y gamma=y
    %
    %Esta función se encarga de dibujar las trayectorias por para 
    %utilizando a beta y a gamma para saber en que posicion ponerlas
    %exactamente
    
    %Se definen cuantos puntos se quieren calcular de la trayectoria.
    %Asimismo, se definen los vectores P que contendran la posicion de este
    %punto respecto al universal
    puntos=20; P=zeros(3,puntos+1); P2=zeros(3,puntos+1); 
    P3=zeros(3,puntos+1); P4=zeros(3,puntos+1);
    
    %Se hace un for para calcular los 'n' puntos definidos arriba
    for i=1:puntos+1
        %Se obtiene un estilo de alfa, que es el 'porcentaje' de la
        %trayectoria circular
        t=2*pi*i/puntos;
        
        % Se calculan todos los 'n' puntos de la trayectoria por pata
        %
        %Aqui se obtienen los puntos de cada trayectoria (LugarDeseado),
        %pero estos puntos deben de multiplicarse por la rotación en z de
        %la gamma correspondiente para así ponerla en la posición correcta.
        %Asimismo, se le suma el desplazamiento del U al origen del eje 0
        %de la pata correspondiente.
        %
        %gamma i+1 = gamma i + pi para que corresponda a la siguiente pata,
        %esto funciona así ya que son 4 patas.
        P(:,i) =rotz(y)          *LugarDeseado(t,b,y)          +(rotz(y)         *[3; 0; 0]);
        P2(:,i)=rotz(y+pi/2)     *LugarDeseado(t,b,y+pi/2)     +(rotz(y+pi/2)    *[3; 0; 0]);
        P3(:,i)=rotz(y+pi)       *LugarDeseado(t,b,y+pi)       +(rotz(y+pi)      *[3; 0; 0]);
        P4(:,i)=rotz(y+(3*pi/2)) *LugarDeseado(t,b,y+(3*pi/2)) +(rotz(y+(3*pi/2))*[3; 0; 0]);
    end
    % Se grafican las trayectorias en rojo
    hold on
    plot3(P(1,:),P(2,:),P(3,:),'r-');
    hold on
    plot3(P2(1,:),P2(2,:),P2(3,:),'r-');
    hold on
    plot3(P3(1,:),P3(2,:),P3(3,:),'r-');
    hold on
    plot3(P4(1,:),P4(2,:),P4(3,:),'r-');
    hold off
    view(140,140)
    drawnow
end

function P=LugarDeseado(a,b,y)
    %Esta función recibe a=alfa, b=beta, y=gamma
    %
    %Se encarga de calcular el punto i visto desde el eje de Q que es el
    %propio eje en donde se traza la trayectoria. El beta determina que tan
    %rotado debe estar este eje en relación al eje universal. Recordemos
    %que la trayectoria se dibuja en el plano y-z. El y es el angulo que
    %existe entre Xu y X0i (eje 0 de la pata i).
    
    %Se determinan en que posición respecto al eje 0i se quieren dibujar
    %hacer la trayectoria circular. (Centro del circulo)
    dx=15; dy=0; dz=-10;  
    
    %Se determinan los radios de z & y. Ry = ancho. Rz = alto.
    ry=4; rz=6;
    
    %Se calcula el punto de la trayectoria respecto al eje Q y se le suman 
    %los valores de traslación respecto al eje 0i
    P=rotz(b-y)*[0;ry*cos(a);max(0,rz*sin(a))]+[dx;dy;dz];
end

function Theta=IK_pata(Pd)
    % Recibe un punto respecto al eje 0i y regresa los valores articulares.
    %
    %Se encarga de calcular los valores articulares por medio de la IK
    %obtenido previamente al hacer el análisis de FK e IK.
  
    %Valores del largo de los eslabones
    L1=6.325; L2=10.05; L3=6.1;
    
    %Calculo de los valores articulares
    q1=atan2(Pd(2),Pd(1));
    c1=cos(q1); s1=sin(q1);
    s3=(L2^2+L3^2-Pd(3)^2-(c1*Pd(1)+s1*Pd(2)-L1)^2)/(2*L2*L3);
    if 1-s3^2<0
        disp('punto inalcanzable.');s3=1;
    end
    q3=atan2(s3,sqrt(1-s3^2));
    c3=cos(q3);
    q2=atan2(Pd(3),c1*Pd(1)+s1*Pd(2)-L1)-atan2(-L3*c3,L2-L3*s3);
    Theta=[q1 q2 q3];
end

function MHDH=MaTransHomoDH(o, d, alfa, a)
    %Esta es una funcion que permite obtener la Matriz de Transformación 
    %Homogénea de Denavit-Hartenberg. 
    %o = omega i
    %d = distancia i
    %alfa = alfa i-1
    %a = a i-1
    
    MHDH = [cos(o)           -sin(o)           0          a
            sin(o)*cos(alfa) cos(o)*cos(alfa) -sin(alfa) -d*sin(alfa)
            sin(o)*sin(alfa) cos(o)*sin(alfa)  cos(alfa)  d*cos(alfa)
            0                0                 0          1];
        
    if ~isnumeric([o,d,alfa,a])
        MHDH=simplify(MHDH);
    end
end