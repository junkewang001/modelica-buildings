within ControlSequence;
package DefrostCycle

  block DefCycCon "RTU coil staging control loop"

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

    Buildings.Controls.OBC.CDL.Interfaces.RealOutput EleHeaCom(
      final min=0,
      final max=1,
      final unit="1") if have_heaCoi "Electric heater command" annotation (Placement(
          transformation(extent={{100,-14},{128,14}}), iconTransformation(
            extent={{100,-20},{140,20}})));
    Buildings.Controls.OBC.CDL.Interfaces.RealInput TCoiSur(
      final unit="K",
      final displayUnit="degC",
      final quantity="ThermodynamicTemperature")
      "Measured coil surface temperature"
                                        annotation (Placement(transformation(
            extent={{-124,-52},{-100,-28}}),
                                           iconTransformation(extent={{-140,-60},
              {-100,-20}})));
    Buildings.Controls.OBC.CDL.Interfaces.RealInput TOut(
      final unit="K",
      displayUnit="degC",
      final quantity="ThermodynamicTemperature") "Outdoor air temperature"
      annotation (Placement(transformation(extent={{-124,68},{-100,92}}),
          iconTransformation(extent={{-140,60},{-100,100}})));
    Buildings.Controls.OBC.CDL.Interfaces.RealInput TCoiSurSet(
      final unit="K",
      final displayUnit="degC",
      final quantity="ThermodynamicTemperature")
      "Predefined coil surface temperature setpoint" annotation (Placement(
          transformation(extent={{-124,-92},{-100,-68}}), iconTransformation(
            extent={{-140,-100},{-100,-60}})));
    Buildings.Controls.OBC.CDL.Interfaces.RealOutput OutFanCom(
      final min=0,
      final max=1,
      final unit="1") "Outdoor fan commanded speed" annotation (Placement(
          transformation(extent={{100,26},{128,54}}), iconTransformation(extent
            ={{100,20},{140,60}})));
    Buildings.Controls.OBC.CDL.Interfaces.RealInput TOutLoc(
      final unit="K",
      displayUnit="degC",
      final quantity="ThermodynamicTemperature")
      "Predefined outdoor lockout temperature setpoint" annotation (Placement(
          transformation(extent={{-124,28},{-100,52}}), iconTransformation(
            extent={{-140,20},{-100,60}})));
    Buildings.Controls.OBC.CDL.Interfaces.BooleanInput uCoiAva[nCoi]
      "DX coil current availability status vector" annotation (Placement(
          transformation(extent={{-124,-12},{-100,12}}), iconTransformation(
            extent={{-140,-20},{-100,20}})));
    Buildings.Controls.OBC.CDL.Interfaces.BooleanOutput yOpeMod[nCoi]
      "DX coil operation mode" annotation (Placement(transformation(extent={{
              100,-54},{128,-26}}), iconTransformation(extent={{100,-60},{140,
              -20}})));
  equation

    connect(OutFanCom, OutFanCom)
      annotation (Line(points={{114,40},{114,40}}, color={0,0,127}));
  annotation (
    defaultComponentName = "supSig",
    Icon(coordinateSystem(preserveAspectRatio=false), graphics={
          Rectangle(
          extent={{-100,-100},{100,102}},
          lineColor={0,0,127},
          fillColor={255,255,255},
          fillPattern=FillPattern.Solid),
          Text(
            extent={{-96,8},{-50,-8}},
            textColor={0,0,127},
            pattern=LinePattern.Dash,
            textString="uCoiAva[nCoi]"),
          Text(
            extent={{-96,-72},{-60,-88}},
            textColor={0,0,127},
            pattern=LinePattern.Dash,
            textString="TCoiSurSet"),
          Text(
            extent={{60,8},{98,-4}},
            textColor={0,0,127},
            pattern=LinePattern.Dash,
            visible=have_heaCoi,
            textString="EleHeaCom"),
          Text(
            extent={{76,46},{98,34}},
            textColor={0,0,127},
            pattern=LinePattern.Dash,
            textString="OutFanCom"),
          Text(
            extent={{64,-32},{98,-46}},
            textColor={0,0,127},
            pattern=LinePattern.Dash,
            textString="yOpeMod[nCoi]"),
          Text(
            extent={{-104,86},{-64,72}},
            textColor={0,0,127},
            pattern=LinePattern.Dash,
            textString="TOut"),
          Text(
            extent={{-114,136},{106,98}},
            textColor={0,0,255},
            textString="%name"),
          Text(
            extent={{-98,48},{-58,34}},
            textColor={0,0,127},
            pattern=LinePattern.Dash,
            textString="TOutLock"),
          Text(
            extent={{-96,-32},{-56,-46}},
            textColor={0,0,127},
            pattern=LinePattern.Dash,
            textString="TCoiSur")}),
                                  Documentation(info="<html>
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
  end DefCycCon;
end DefrostCycle;
