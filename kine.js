var Pi = Math.PI;
var Yoffset;
var D;
var d;
var R;
var L1 = 110; //height of the first link from surface to 2nd joint position
var L2 = 105; //length of the second link from 2nd joint to 3rd joint
var L3 = 77; //length of the 3rd joint to 4th joint
var L4 = 65; //length of the 4th joint to the tip of the arm (gripper)

var X_End_Effector; //x axis coordinate of the gripper
var Y_End_Effector; //y axis coordinate of the gripper
var Z_End_Effector; //z axis coordinate of the gripper

var X, Y;
var Z;
var b6, b7;

var obj = {};

var alpha1;
var alpha2;
var Theta_2;  
var Theta_4;  
var Theta_1;  
var Theta_3; 
var angles;

var xval, in_min, out_min, out_max, in_max;

window.addEventListener('load', (event) => {
    //X_End_Effector = prompt();
    //console.log("X= "+ X_End_Effector);
    //Y_End_Effector = prompt();
    //console.log("Y= "+ Y_End_Effector);
    //Z_End_Effector = prompt();
    //console.log("Z= "+ Z_End_Effector);
    gpad();
    
    
    //console.log(ang);
  });

function Rumus_IK(X_End_Effector, Y_End_Effector, Z_End_Effector)
{
  if (X_End_Effector > 0 && Y_End_Effector >= L1)
  {
    D = Math.sqrt(Math.pow(X_End_Effector,2) + Math.pow(Z_End_Effector,2));
    Theta_1 = (Math.atan(Z_End_Effector/X_End_Effector))*(180.00/Pi); //theta 1
    d = D - L4;
    Yoffset = Y_End_Effector - L1;
    R = Math.sqrt(Math.pow(d,2) + Math.pow(Yoffset,2));
    alpha1 = (Math.acos(d/R))*(180.00/Pi);
    alpha2 = (Math.acos((Math.pow(L2,2) + Math.pow(R,2) - Math.pow(L3,2))/(2*L2*R)))*(180.00/Pi);
    Theta_2 = (alpha1 + alpha2); //theta 2
    Theta_3 = ((Math.acos((Math.pow(L2,2) + Math.pow(L3,2) - Math.pow(R,2))/(2*L2*L3)))*(180.00/Pi)); //theta 3
    Theta_4 = 180.00 - ((180.00 - (alpha2 + Theta_3)) - alpha1); //theta 4
  }
  else if (X_End_Effector > 0 && Y_End_Effector <= L1)
  {
    D = Math.sqrt(Math.pow(X_End_Effector,2) + Math.pow(Z_End_Effector,2));
    Theta_1 = (Math.atan(Z_End_Effector/X_End_Effector))*(180.00/Pi); //theta 1
    d = D - L4;
    Yoffset = Y_End_Effector - L1;
    R = Math.sqrt(Math.pow(d,2) + Math.pow(Yoffset,2));
    alpha1 = (Math.acos(d/R))*(180.00/Pi);
    alpha2 = (Math.acos((Math.pow(L2,2) + Math.pow(R,2) - Math.pow(L3,2))/(2*L2*R)))*(180.00/Pi);
    Theta_2 = (alpha2 - alpha1); //theta 2
    Theta_3 = ((Math.acos((Math.pow(L2,2) + Math.pow(L3,2) - Math.pow(R,2))/(2*L2*L3)))*(180.00/Pi)); //theta 3
    Theta_4 = 180.00 - ((180.00 - (alpha2 + Theta_3)) + alpha1); //theta 4
  }
  else if (X_End_Effector == 0 && Y_End_Effector >= L1)
  {
    D = Math.sqrt(Math.pow(X_End_Effector,2) + Math.pow(Z_End_Effector,2));
    Theta_1 = 90.00; //theta 1
    d = D - L4;
    Yoffset = Y_End_Effector - L1;
    R = Math.sqrt(Math.pow(d,2) + Math.pow(Yoffset,2));
    alpha1 = (Math.acos(d/R))*(180.00/Pi);
    alpha2 = (Math.acos((Math.pow(L2,2) + Math.pow(R,2) - Math.pow(L3,2))/(2*L2*R)))*(180.00/Pi);
    Theta_2 = (alpha1 + alpha2); //theta 2
    Theta_3 = ((Math.acos((Math.pow(L2,2) + Math.pow(L3,2) - Math.pow(R,2))/(2*L2*L3)))*(180.00/Pi)); //theta 3
    Theta_4 = 180.00 - ((180.00 - (alpha2 + Theta_3)) - alpha1); //theta 4
  }
  else if (X_End_Effector == 0 && Y_End_Effector <= L1)
  {
    D = Math.sqrt(Math.pow(X_End_Effector,2) + Math.pow(Z_End_Effector,2));
    Theta_1 = 90.00; //theta 1
    d = D - L4;
    Yoffset = Y_End_Effector - L1;
    R = Math.sqrt(Math.pow(d,2) + Math.pow(Yoffset,2));
    alpha1 = (Math.acos(d/R))*(180.00/Pi);
    alpha2 = (Math.acos((Math.pow(L2,2) + Math.pow(R,2) - Math.pow(L3,2))/(2*L2*R)))*(180.00/Pi);
    Theta_2 = (alpha2 - alpha1); //theta 2
    Theta_3 = ((Math.acos((Math.pow(L2,2) + Math.pow(L3,2) - Math.pow(R,2))/(2*L2*L3)))*(180.00/Pi)); //theta 3
    Theta_4 = 180.00 - ((180.00 - (alpha2 + Theta_3)) + alpha1); //theta 4
  }
  else if (X_End_Effector < 0 && Y_End_Effector >= L1)
  {
    D = Math.sqrt(Math.pow(X_End_Effector,2) + Math.pow(Z_End_Effector,2));
    Theta_1 = 90.00 + (90.00 - Math.abs((Math.atan(Z_End_Effector/X_End_Effector))*(180.00/Pi))); //theta 1
    d = D - L4;
    Yoffset = Y_End_Effector - L1;
    R = Math.sqrt(Math.pow(d,2) + Math.pow(Yoffset,2));
    alpha1 = (Math.acos(d/R))*(180.00/Pi);
    alpha2 = (Math.acos((Math.pow(L2,2) + Math.pow(R,2) - Math.pow(L3,2))/(2*L2*R)))*(180.00/Pi);
    Theta_2 = (alpha1 + alpha2); //theta 2
    Theta_3 = ((Math.acos((Math.pow(L2,2) + Math.pow(L3,2) - Math.pow(R,2))/(2*L2*L3)))*(180.00/Pi)); //theta 3
    Theta_4 = 180.00 - ((180.00 - (alpha2 + Theta_3)) - alpha1); //theta 4
  }
  else if (X_End_Effector < 0 && Y_End_Effector <= L1)
  {
    D = Math.sqrt(Math.pow(X_End_Effector,2) + Math.pow(Z_End_Effector,2));
    Theta_1 = 90.00 + (90.00 - Math.abs((Math.atan(Z_End_Effector/X_End_Effector))*(180.00/Pi))); //theta 1
    d = D - L4;
    Yoffset = Y_End_Effector - L1;
    R = Math.sqrt(Math.pow(d,2) + Math.pow(Yoffset,2));
    alpha1 = (Math.acos(d/R))*(180.00/Pi);
    alpha2 = (Math.acos((Math.pow(L2,2) + Math.pow(R,2) - Math.pow(L3,2))/(2*L2*R)))*(180.00/Pi);
    Theta_2 = (alpha2 - alpha1); //theta 2
    Theta_3 = ((Math.acos((Math.pow(L2,2) + Math.pow(L3,2) - Math.pow(R,2))/(2*L2*L3)))*(180.00/Pi)); //theta 3
    Theta_4 = 180.00 - ((180.00 - (alpha2 + Theta_3)) + alpha1); //theta 4
  }
  b6 = map(b6, 0, 1, 0, 180);
  b7 = map(b7, 0, 1, 90, 180);
  var TempAngle;
  angles = [Theta_1, Theta_2, Theta_3, Theta_4];
  console.log(angles);
    for (var angle in angles){
        TempAngle = (angles[angle]);
        if (TempAngle<0){
            if(Math.abs(TempAngle)>180){
                TempAngle = TempAngle + 360;
                //console.log(TempAngle);
                angles[angle] = TempAngle;
            }
            else{
                TempAngle = TempAngle +180;
                //console.log(TempAngle);
                angles[angle] = TempAngle;
            }
        }
        else if (TempAngle>180){
        	TempAngle = TempAngle-180;
            //console.log(TempAngle);
            angles[angle] = TempAngle;
        }
        else{
        //console.log(TempAngle);
        angles[angle] = TempAngle;
        }
    }
    angles =[angles[0], angles[1], angles[2], angles[3], b6, b7];
    console.log(angles)
    
    return angles;
}

function gpad(){
        setInterval(function(){
        const gamepad = navigator.getGamepads()[0];
        if(!gamepad)  return;
        X = gamepad.axes[0].toFixed(3);
        Y = gamepad.axes[1].toFixed(3);
        Z = gamepad.axes[3].toFixed(3);
        b6 = gamepad.buttons[6].value.toFixed(3);
        b7 = gamepad.buttons[7].value.toFixed(3)
        X_End_Effector = map(X, -1, 1, -209.5, 209.5);
        Y_End_Effector = map(Y, -1, 1, -59.5, 279.5);
        Z_End_Effector = map(Z, -1, 1, -209.5, 209.5);
        
        a = Rumus_IK(X_End_Effector, Y_End_Effector, Z_End_Effector);
        document.getElementById("obj").innerHTML = a;
        document.getElementById("t1").innerHTML = a[0];
        document.getElementById("t2").innerHTML = a[1];
        document.getElementById("t3").innerHTML = a[2];
        document.getElementById("t4").innerHTML = a[3];
        document.getElementById("t5").innerHTML = a[4];
        document.getElementById("t6").innerHTML = a[5];
        }, 50);
}

window.addEventListener("gamepadconnected", (event) => {
    console.log("A gamepad connected:");
    console.log(event.gamepad);
    console.log(event.gamepad.axes.length)
  });
  
  window.addEventListener("gamepaddisconnected", (event) => {
    console.log("A gamepad disconnected:");
    console.log(event.gamepad);
  });

function map(xval, in_min, in_max, out_min, out_max) {
    return (xval - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
  }