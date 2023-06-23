within Buildings.Fluid.ZoneEquipment;
package WindowAC "Package with models for the window air conditioner"
  annotation (Icon(graphics={
        Rectangle(
          lineColor={200,200,200},
          fillColor={248,248,248},
          fillPattern=FillPattern.HorizontalCylinder,
          extent={{-100,-100},{100,100}},
          radius=25.0),
        Rectangle(
          lineColor={128,128,128},
          extent={{-100,-100},{100,100}},
          radius=25.0),
        Ellipse(
          origin={10,10},
          fillColor={76,76,76},
          pattern=LinePattern.None,
          fillPattern=FillPattern.Solid,
          extent={{-80.0,-80.0},{-20.0,-20.0}}),
        Ellipse(
          origin={10,10},
          pattern=LinePattern.None,
          fillPattern=FillPattern.Solid,
          extent={{0.0,-80.0},{60.0,-20.0}}),
        Ellipse(
          origin={10,10},
          fillColor={128,128,128},
          pattern=LinePattern.None,
          fillPattern=FillPattern.Solid,
          extent={{0.0,0.0},{60.0,60.0}}),
        Ellipse(
          origin={10,10},
          lineColor={128,128,128},
          fillColor={255,255,255},
          fillPattern=FillPattern.Solid,
          extent={{-80.0,0.0},{-20.0,60.0}})}),
  Documentation(info="<html>
  This package contains models for the window air conditioner unit, including the system model
  <a href=\"modelica://Buildings.Fluid.ZoneEquipment.WindowAC.WindowAC\">
  Buildings.Fluid.ZoneEquipment.WindowAC.WindowAC</a>, example model in 
  <a href=\"modelica://Buildings.Fluid.ZoneEquipment.WindowAC.Examples\">
  Buildings.Fluid.ZoneEquipment.WindowAC.Examples</a>, and validation model in 
  <a href=\"modelica://Buildings.Fluid.ZoneEquipment.WindowAC.Validation\">
  Buildings.Fluid.ZoneEquipment.WindowAC.Validation</a>. 
  </html>"));
end WindowAC;
