%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%--Programmer: Crasun Jans (Janak Panthi)-----------------------%%
%%--Date: 01-30-2022---------------------------------------------%%
%%--Function name: tM--------------------------------------------%%
%%--Description:This function returns a 4x4 transformation matrix%%
%%--based on the following arguments:----------------------------%%
%%--axis: x,y,or z rotation axis---------------------------------%%
%%--theta: rotation angle in radian------------------------------%%
%%--pos: position vector-----------------------------------------%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

function tM = tM(axis, theta, pos)
%constants
RO3 = zeros(1,3);

%if pos argument is not sent, assume no translation
if ~exist('pos','var')

          pos = zeros(3,1);
          
          if ~exist('theta','var')

              %to return 4x4 identity
              if isequal(axis,eye(3))
                  tM = [eye(3) pos; 
                        RO3  1 ];
                  return;

               %to return identity and translation (i.e. no rotation)
              elseif isequal(size(axis),[3 1])
                    pos = axis;
                    tM = [eye(3) pos; 
                    RO3  1 ];
                    return;
              end


          end
end

%if all three arguments are sent, compute accordingly
if strcmpi(axis,'z')%case insensitive check for z
rot = [cos(theta) -sin(theta) 0;sin(theta) cos(theta) 0; 0 0 1 ];%for rotation about the z-axis
tM = [rot pos; 
      RO3  1 ];

elseif strcmpi(axis,'y')
rot = [cos(theta) 0 sin(theta) ; 0 1 0; -sin(theta) 0 cos(theta) ];%for rotation about the y-axis
tM = [rot pos; 
      RO3  1 ];

elseif strcmpi(axis,'x')
rot = [1 0 0; 0 cos(theta) -sin(theta);0 sin(theta) cos(theta) ];%for rotation about the x-axis
tM = [rot pos; 
      RO3  1 ];
end
