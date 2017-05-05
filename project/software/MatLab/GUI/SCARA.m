classdef SCARA < TimeSteppingRigidBodyManipulator
% SCARA Manipulator

  methods
    
    function obj=SCARA(urdf,options)
      %checks the urdf address
      typecheck(urdf,'char');
      
      %options for the SCARA implementation
      options = struct();
      options.dt = 0.001;
      options.floating = false;
      options.base_offset = [-0.5, 0, 1.5]';
      options.base_rpy = [pi/2, pi/2, 0]';
      options.terrain = RigidBodyFlatTerrain;
      
      %loads the manipulator URDF
      obj = obj@TimeSteppingRigidBodyManipulator('',options.dt,options);
      obj = obj.addRobotFromURDF(urdf,options.base_offset,options.base_rpy,options);
    end
    
    function obj = compile(obj)
      %Compiles the manipulator
      obj = compile@TimeSteppingRigidBodyManipulator(obj);
    end

    function xstar = home(obj)
      %returns the home state of the robot
      xstar = Point(getStateFrame(obj));
      xstar.base_yaw = 0;
      xstar.base_roll = 0;
      xstar.base_pitch = 0;
    end

    function [v_x, v_y] = translate(r, p)
      %assigns v_x based on the roll angle
      if r < -1.01*pi/2
      elseif r < -0.75*pi/2
        v_x = -0.57*VMAX;
      elseif r < -0.50*pi/2
        v_x = -0.50*VMAX;
      elseif r < -0.25*pi/2
        v_x = -0.25*VMAX;
      elseif r < 0
        v_x = 0.00;
      elseif r < 0.25*pi/2
        v_x = 0.00;
      elseif r < 0.50*pi/2
        v_x = 0.25*VMAX;
      elseif r < 0.75*pi/2
        v_x = 0.50*VMAX;
      elseif r < 1.00*pi/2
        v_x = 0.75*VMAX;
      else
        v_x = 1.00*VMAX;
      end

      %assigns v_y based on the pitch angle
      if p < -1.01*pi/2
        v_y = -VMAX;
      elseif p < -0.75*pi/2
        v_y = -0.75*VMAX;
      elseif p < -0.50*pi/2
        v_y = -0.50*VMAX;
      elseif p < -0.25*pi/2
        v_y = -0.25*VMAX;
      elseif P < 0
        v_y = 0.00;
      elseif P < 0.25*pi/2
        v_y = 0.00;
      elseif p < 0.50*pi/2
        v_y = 0.25*VMAX;
      elseif p < 0.75*pi/2
        v_y = 0.50*VMAX;
      elseif p < 1.00*pi/2
        v_y = 0.75*VMAX;
      else
        v_y = VMAX;
      end
    end

    function q = IK(v_x, v_y, d_z)
      %updates the x, y and z positions
      x += v_x*dt;
      y += v_y*dt;
      z += d_z;
      
      if(z > z_max)
        z = z_max;
      elseif (z < z_min)
        z = z_min
      end
      
      if(y > y_max)
        y = y_max;
      elseif (y < y_min)
        y = y_min
      end

      if(x > x_max)
        x = x_max;
      elseif (x < x_min)
        x = x_min
      end

      q_3 = z;
      q_2 = acos((x^2 + y^2 + L_1^2 + L_2^2)/(2*L_1*L_2));
      q_1 = atan2(y,x) - acos((x^2 + y^2 + L_1^2 + L_2^2)/(2*L_1*sqrt(x^2 + y^2)));
    end

    function xtraj = interpolate(obj, q_0, q_f)
      %generates the time vector
      ts = linspace(0,dt,2);

      %the q vector
      q = [q_0, q_f];

      %interpolates the splines
      x = zeros(getNumStates(obj),length(ts));
      x(1:getNumPositions(obj),:) = q;
      xtraj = PPTrajectory(spline(ts, x));
      xtraj = xtraj.setOutputFrame(obj.getStateFrame());
    end

    function [r, p, d_z] = getRPZ()
      %initializes the port
      s = serial(obj.port, 'Baudrate', 115200);
      fopen(s);
      %reads the variables from the port
      r = fread(s,4,'float');
      p = fread(s,4,'float');
      d_z = fread(s,2,'int16');
      %close the port
      fclose(s);
    end
    
    function runSim(obj)
      %loads the robot from the URDF
      r = obj.SCARA('scara.urdf');
      r.compile();
      q_0 = r.home();

      %builds a visualizer for the robot
      v = r.constructVisualizer();
      v.draw(q_0);

      while 1
        %reads the filtered signals
        [r, p, d_z] = obj.getRPZ();
        %translates the angles to velocities
        [v_x, v_y] = obj.translate(r,p);
        %runs the inverse kinematics
        q = obj.IK(v_x, v_y, d_z);
        %interpolates the splines for the trajectory
        xtraj = obj.interpolate(q_0,q);
        %plays the generated trajectory
        v.playback(xtraj);
        %updates the q vector
        q_0 = q;
      end
    end
  end

  properties (SetAccess = protected, GetAccess = public)
    base_rpy;
    base_offset;
    dt = 0.029;
    x = 0;
    y = 0;
    z = 0;
    L_1 = 10;
    L_2 = 10;
    Port = '/dev/tty.KeySerial1');
  end
end
