clc;
clear;

% creacion de instrumento udp
% remote_ip_address = 192.168.4.1 ; remote_port = 1024
% local_ip_address = 192.168.4.2 ; local_port = 1024

obj1 = udp('192.168.4.1', 1024);

% configuracion de instrumento udp
set(obj1, 'DatagramTerminateMode', 'on'); % Para leer datos de forma continua
set(obj1, 'LocalPort', 1024);
set(obj1, 'LocalPortMode', 'manual');
set(obj1, 'ByteOrder', 'littleEndian');

% abrir instrumento udp
fopen(obj1);

% comunicacion utilizando 'obj1'
i=1;
while(1)
    data  = fread(obj1, 2, 'uint8'); % distance --> 2Bytes
    data1 = fread(obj1, 4, 'uint8'); % Roll Angle --> 4Bytes
    data2 = fread(obj1, 4, 'uint8'); % Pitch Angle --> 4Bytes
        
    data  = uint8(data');
    data1 = uint8(data1');
    data2 = uint8(data2');
       
    d(i) = typecast(fliplr(data), 'uint16')
    r(i) = typecast(fliplr(data1), 'single')
    p(i) = typecast(fliplr(data2), 'single')

    i=i+1; % incrementa contador
end
%% cerrar y eliminar instrumento udp
fclose(obj1);
delete(obj1);
%% Graficar roll, pitch y distance
figure(1), plot(p,'r'), axis([0 length(p) -pi/2 pi/2])
title('Figure 1 - Pitch'), xlabel('dt'), ylabel('angle'), grid on
figure(2), plot(r,'b'), axis([0 length(r) -pi/2 pi/2])
title('Figure 2 - Roll'), xlabel('dt'), ylabel('angle'), grid on
figure(3), plot(d,'g'), axis([0 length(d) 0 100])
title('Figure 3 - distance'), xlabel('dt'), ylabel('angle'), grid on