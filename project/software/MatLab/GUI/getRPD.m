fclose(s);
s = serial('/dev/cu.usbserial-FT3W5E11','BaudRate', 115200);
fopen(s);

roll = [];
pitch = [];

i = 1;
t = tic();
while 1
  toc(t)
  t = tic();
  di = fread(s,2,'uint8'); 
  ri = fread(s,4,'uint8'); pi = fread(s,4,'uint8');
  %di = uint8(di');
  ri = uint8(ri');
  pi = uint8(pi');
  %d = typecast(fliplr(di) , 'uint16');
  r = typecast(fliplr(ri) , 'single');
  p = typecast(fliplr(pi) , 'single');
  %d = typecast(fliplr(di) , 'uint16');
  
  display(sprintf('roll = %f',r));
  display(sprintf('pitch = %f',p));
  
  pitch = [pitch, p];
  roll = [roll, r];
  
  %display(sprintf('distance = %d \n',d))
end

fclose(s);

