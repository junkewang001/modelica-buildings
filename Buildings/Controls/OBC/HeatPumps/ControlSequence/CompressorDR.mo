within ControlSequence;
package CompressorDR

  block ComDRCon "RTU coil staging control loop"

    parameter Boolean have_heaCoi=true
      "True: the AHU has heating coil. It could be the hot water coil, or the electric heating coil";
    parameter Buildings.Controls.OBC.CDL.Types.SimpleController controllerType=
        Buildings.Controls.OBC.CDL.Types.SimpleController.PI
      "Type of controller for supply air temperature signal";
    parameter Real kTSup(final unit="1/K")=0.05
      "Gain of controller for supply air temperature signal";
    parameter Real TiTSup(
      final unit="s",
      final quantity="Time")=600
      "Time constant of integrator block for supply temperature control signal"
      annotation(Dialog(
        enable=controllerType == Buildings.Controls.OBC.CDL.Types.SimpleController.PI
            or controllerType == Buildings.Controls.OBC.CDL.Types.SimpleController.PID));
    parameter Real TdTSup(
      final unit="s",
      final quantity="Time")=0.1
      "Time constant of derivative block for supply temperature control signal"
      annotation(Dialog(enable=controllerType == Buildings.Controls.OBC.CDL.Types.SimpleController.PD
                            or controllerType == Buildings.Controls.OBC.CDL.Types.SimpleController.PID));
    parameter Real uHea_max(
      final min=-0.9,
      final unit="1")=-0.25
      "Upper limit of controller signal when heating coil is off. Require -1 < uHea_max < uCoo_min < 1.";
    parameter Real uCoo_min(
      final max=0.9,
      final unit="1")=0.25
      "Lower limit of controller signal when cooling coil is off. Require -1 < uHea_max < uCoo_min < 1.";

    Buildings.Controls.OBC.CDL.Interfaces.IntegerInput uHeaDemLimLev
      "Heating demand limit level"
      annotation (Placement(transformation(extent={{-124,36},{-100,60}}),
          iconTransformation(extent={{-140,20},{-100,60}})));
    Buildings.Controls.OBC.CDL.Interfaces.IntegerInput uCooDemLimLev
      "Cooling demand limit level" annotation (Placement(transformation(extent=
              {{-124,-52},{-100,-28}}), iconTransformation(extent={{-140,-68},{
              -100,-28}})));
    Buildings.Controls.OBC.CDL.Interfaces.RealOutput yComSpe(
      min=0,
      max=1,
      final unit="1") "Compressor commanded speed" annotation (Placement(
          transformation(extent={{100,-14},{128,14}}), iconTransformation(
            extent={{100,-20},{140,20}})));
  equation

  annotation (
    defaultComponentName = "supSig",
    Icon(coordinateSystem(preserveAspectRatio=false), graphics={
          Rectangle(
          extent={{-100,-100},{100,100}},
          lineColor={0,0,127},
          fillColor={255,255,255},
          fillPattern=FillPattern.Solid),
          Text(
            extent={{-98,48},{-52,32}},
            textColor={0,0,127},
            pattern=LinePattern.Dash,
            textString="uHeaDemLimLev"),
          Text(
            extent={{-98,-32},{-62,-48}},
            textColor={0,0,127},
            pattern=LinePattern.Dash,
            textString="uCooDemLimLev"),
          Text(
            extent={{60,6},{98,-6}},
            textColor={0,0,127},
            pattern=LinePattern.Dash,
            visible=have_heaCoi,
            textString="yComSpe"),
          Text(
            extent={{-114,136},{106,98}},
            textColor={0,0,255},
            textString="%name")}),Documentation(info="<html>
<p>
Block that outputs the supply temperature control loop signal,
and the coil valve postions for VAV system with multiple zones,
implemented according to Section 5.16.2.3 of the ASHRAE Guideline G36, May 2020.
</p>
<p>
The supply air temperature control loop signal <code>uTSup</code>
is computed using a PI controller that tracks the supply air temperature
setpoint <code>TSupSet</code>.
If the fan is off, then <code>uTSup = 0</code>.
</p>
<p>
Heating valve control signal (or modulating electric heating
coil if applicable) <code>yHeaCoi</code> and cooling valve control signal <code>yCooCoi</code>
are sequenced based on the supply air temperature control loop signal <code>uTSup</code>.
From <code>uTSup = uHea_max</code> to <code>uTSup = -1</code>,
<code>yHeaCoi</code> increases linearly from <i>0</i> to <i>1</i>.
Similarly, <code>uTSup = uCoo_min</code> to <code>uTSup = +1</code>,
<code>yCooCoi</code> increases linearly from <i>0</i> to <i>1</i>.
</p>

<p align=\"center\">
<img alt=\"Image of supply temperature loop\"
src=\"modelica://Buildings/Resources/Images/Controls/OBC/ASHRAE/G36/AHUs/MultiZone/VAV/SetPoints/SupTemLoop.png\"/>
</p>

<p>
The output <code>uTSup</code> can be used in a controller for the economizer.
</p>
</html>",
  revisions="<html>
<ul>
<li>
August 1, 2020, by Jianjun Hu:<br/>
Updated according to ASHRAE G36 official release.
</li>
<li>
November 1, 2017, by Jianjun Hu:<br/>
First implementation.
</li>
</ul>
</html>"));
  end ComDRCon;
end CompressorDR;
