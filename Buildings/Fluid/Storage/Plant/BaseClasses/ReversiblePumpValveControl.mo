within Buildings.Fluid.Storage.Plant.BaseClasses;
block ReversiblePumpValveControl
  "Control block for the secondary pump-valve group"
  extends Modelica.Blocks.Icons.Block;

  Modelica.Blocks.Sources.Constant conOne(k=1) "Constant y = 1"
    annotation (Placement(transformation(extent={{-100,-90},{-80,-70}})));
  Modelica.Blocks.Sources.Constant conZero(k=0) "Constant y = 0"
    annotation (Placement(transformation(extent={{-100,-120},{-80,-100}})));
  Buildings.Controls.Continuous.LimPID conPI_pum2(
    controllerType=Modelica.Blocks.Types.SimpleController.PI,
    Td=1,
    k=5,
    Ti=50)   "PI controller for pum2" annotation (Placement(transformation(
        extent={{-10,10},{10,-10}},
        rotation=270,
        origin={-70,70})));
  Modelica.Blocks.Math.Gain gaiPum2(k=1) "Gain" annotation (Placement(
        transformation(
        extent={{10,-10},{-10,10}},
        rotation=90,
        origin={-70,30})));
  Buildings.Controls.Continuous.LimPID conPI_val2(
    controllerType=Modelica.Blocks.Types.SimpleController.PI,
    Td=1,
    k=5,
    Ti=50,
    reverseActing=false)
             "PI controller for val2" annotation (Placement(transformation(
        extent={{-10,10},{10,-10}},
        rotation=270,
        origin={-10,68})));
  Modelica.Blocks.Math.Gain gaiVal2(k=1) "Gain" annotation (Placement(
        transformation(
        extent={{10,-10},{-10,10}},
        rotation=90,
        origin={-10,30})));
  Buildings.Controls.OBC.CDL.Interfaces.BooleanInput booFloDir
    "Flow direction: true = normal; false = reverse" annotation (Placement(
        transformation(extent={{-120,-10},{-100,10}}), iconTransformation(
          extent={{-140,-60},{-100,-20}})));
  Modelica.Blocks.Interfaces.RealInput us_mTan_flow
    "Tank mass flow rate setpoint"   annotation (Placement(transformation(
          extent={{-10,-10},{10,10}},
          rotation=-90,
          origin={-70,110}), iconTransformation(
          extent={{-10,-10},{10,10}},
          rotation=0,
          origin={-110,40})));
  Modelica.Blocks.Interfaces.RealInput um_mTan_flow
      "Measured tank mass flow rate" annotation (Placement(transformation(
          extent={{-10,-10},{10,10}},
          rotation=-90,
          origin={10,110}),  iconTransformation(
          extent={{-10,-10},{10,10}},
          rotation=0,
          origin={-110,80})));
  Buildings.Controls.OBC.CDL.Interfaces.BooleanInput booOnOff
    "Plant output: true = on; false = off"       annotation (Placement(
        transformation(extent={{-120,-30},{-100,-10}}), iconTransformation(
          extent={{-140,-100},{-100,-60}})));
  Modelica.Blocks.Interfaces.RealOutput yPum2 "Normalised speed" annotation (
      Placement(transformation(
        extent={{-10,-10},{10,10}},
        rotation=-90,
        origin={-50,-170}), iconTransformation(
        extent={{-10,-10},{10,10}},
        rotation=-90,
        origin={0,-110})));
  Modelica.Blocks.Interfaces.RealOutput yVal2 "Valve position" annotation (
      Placement(transformation(
        extent={{-10,-10},{10,10}},
        rotation=-90,
        origin={70,-170}), iconTransformation(
        extent={{-10,-10},{10,10}},
        rotation=-90,
        origin={80,-110})));
  Modelica.Blocks.Interfaces.RealOutput yVal1 "Valve position" annotation (
      Placement(transformation(
        extent={{-10,-10},{10,10}},
        rotation=-90,
        origin={10,-170}), iconTransformation(
        extent={{-10,-10},{10,10}},
        rotation=-90,
        origin={40,-110})));
  Modelica.Blocks.Interfaces.RealInput yVal2_actual "Actual position of val2"
    annotation (Placement(transformation(
        extent={{-10,-10},{10,10}},
        rotation=180,
        origin={110,70}), iconTransformation(
        extent={{10,-10},{-10,10}},
        rotation=0,
        origin={110,40})));
  Modelica.Blocks.Interfaces.RealInput yVal1_actual "Actual position of val1"
    annotation (Placement(transformation(
        extent={{-10,-10},{10,10}},
        rotation=180,
        origin={110,30}), iconTransformation(
        extent={{10,-10},{-10,10}},
        rotation=0,
        origin={110,80})));
  Buildings.Controls.OBC.CDL.Continuous.LessThreshold lesThrVal1(t=0.05)
    "yVal1_actual less than threshold"
    annotation (Placement(transformation(extent={{80,20},{60,40}})));
  Buildings.Controls.OBC.CDL.Continuous.Switch swiPum1
    "Switch: true = on; false = off (y=0)." annotation (Placement(
        transformation(
        extent={{-10,-10},{10,10}},
        rotation=-90,
        origin={-50,-130})));
  Buildings.Controls.OBC.CDL.Continuous.Switch swiVal1
    "Switch: true = on (y=1); false = off (y=0)." annotation (Placement(
        transformation(
        extent={{-10,-10},{10,10}},
        rotation=-90,
        origin={10,-130})));
  Buildings.Controls.OBC.CDL.Continuous.Switch swiVal2
    "Switch: true = on; false = off (y=0)." annotation (Placement(
        transformation(
        extent={{-10,-10},{10,10}},
        rotation=-90,
        origin={70,-130})));
  Buildings.Controls.OBC.CDL.Logical.And3 and3Val1
    "Plant on AND normal direction AND val2 closed"     annotation (Placement(
        transformation(
        extent={{-10,-10},{10,10}},
        rotation=-90,
        origin={0,-50})));
  Buildings.Controls.OBC.CDL.Continuous.LessThreshold lesThrVal2(t=0.05)
    "yVal1_actual less than threshold"
    annotation (Placement(transformation(extent={{80,60},{60,80}})));
  Buildings.Controls.OBC.CDL.Logical.Not notFloDirVal2
    "Reverses flow direction signal for val2" annotation (Placement(
        transformation(
        extent={{-10,-10},{10,10}},
        rotation=-90,
        origin={60,-16})));
  Buildings.Controls.OBC.CDL.Logical.And3 and3Pum2
    "Plant on AND normal direction AND val2 closed"     annotation (Placement(
        transformation(
        extent={{-10,-10},{10,10}},
        rotation=-90,
        origin={-60,-50})));
  Buildings.Controls.OBC.CDL.Logical.And andVal2 "Reverse direction AND val1 closed"
    annotation (Placement(transformation(
        extent={{-10,-10},{10,10}},
        rotation=-90,
        origin={60,-50})));
equation

  connect(conPI_pum2.y,gaiPum2. u)
    annotation (Line(points={{-70,59},{-70,42}},   color={0,0,127}));
  connect(conPI_val2.y,gaiVal2. u) annotation (Line(points={{-10,57},{-10,42}},
                                  color={0,0,127}));
    connect(conPI_pum2.u_s, us_mTan_flow)
      annotation (Line(points={{-70,82},{-70,110}}, color={0,0,127}));
    connect(conPI_val2.u_s, us_mTan_flow) annotation (Line(points={{-10,80},{-10,
          88},{-70,88},{-70,110}},   color={0,0,127}));
    connect(conPI_val2.u_m, um_mTan_flow) annotation (Line(points={{2,68},{10,68},
          {10,110}},                         color={0,0,127}));
    connect(conPI_pum2.u_m, um_mTan_flow) annotation (Line(points={{-58,70},{-52,
          70},{-52,94},{10,94},{10,110}},     color={0,0,127}));
  connect(yVal1, yVal1)
    annotation (Line(points={{10,-170},{10,-170}}, color={0,0,127}));
  connect(swiPum1.y, yPum2)
    annotation (Line(points={{-50,-142},{-50,-170}}, color={0,0,127}));
  connect(conZero.y, swiPum1.u3) annotation (Line(points={{-79,-110},{-58,-110},
          {-58,-118}}, color={0,0,127}));
  connect(conZero.y, swiVal1.u3)
    annotation (Line(points={{-79,-110},{2,-110},{2,-118}}, color={0,0,127}));
  connect(swiVal1.y, yVal1)
    annotation (Line(points={{10,-142},{10,-170}}, color={0,0,127}));
  connect(swiVal2.y, yVal2)
    annotation (Line(points={{70,-142},{70,-170}}, color={0,0,127}));
  connect(conZero.y, swiVal2.u3) annotation (Line(points={{-79,-110},{62,-110},{
          62,-118}}, color={0,0,127}));
  connect(conOne.y, swiVal1.u1)
    annotation (Line(points={{-79,-80},{18,-80},{18,-118}}, color={0,0,127}));
  connect(gaiPum2.y, swiPum1.u1) annotation (Line(points={{-70,19},{-70,10},{-42,
          10},{-42,-118}}, color={0,0,127}));
  connect(gaiVal2.y, swiVal2.u1) annotation (Line(points={{-10,19},{-10,10},{78,
          10},{78,-118}}, color={0,0,127}));
  connect(booOnOff, and3Val1.u3) annotation (Line(points={{-110,-20},{-8,-20},{-8,
          -38}}, color={255,0,255}));
  connect(booFloDir, and3Val1.u2) annotation (Line(points={{-110,0},{0,0},{0,-20},
          {2.22045e-15,-20},{2.22045e-15,-38}}, color={255,0,255}));
  connect(lesThrVal2.u, yVal2_actual)
    annotation (Line(points={{82,70},{110,70}}, color={0,0,127}));
  connect(lesThrVal2.y, and3Val1.u1) annotation (Line(points={{58,70},{34,70},{34,
          40},{8,40},{8,-38}}, color={255,0,255}));
  connect(lesThrVal1.u, yVal1_actual)
    annotation (Line(points={{82,30},{110,30}}, color={0,0,127}));
  connect(and3Val1.y, swiVal1.u2) annotation (Line(points={{-2.22045e-15,-62},{-2.22045e-15,
          -70},{10,-70},{10,-118}},              color={255,0,255}));
  connect(notFloDirVal2.u, booFloDir)
    annotation (Line(points={{60,-4},{60,0},{-110,0}}, color={255,0,255}));
  connect(and3Pum2.u3, booOnOff) annotation (Line(points={{-68,-38},{-68,-20},{
          -110,-20}}, color={255,0,255}));
  connect(and3Pum2.u2, booFloDir)
    annotation (Line(points={{-60,-38},{-60,0},{-110,0}}, color={255,0,255}));
  connect(and3Pum2.u1, lesThrVal2.y) annotation (Line(points={{-52,-38},{-52,
          -30},{8,-30},{8,40},{34,40},{34,70},{58,70}}, color={255,0,255}));
  connect(and3Pum2.y, swiPum1.u2) annotation (Line(points={{-60,-62},{-60,-70},
          {-50,-70},{-50,-118}}, color={255,0,255}));
  connect(notFloDirVal2.y, andVal2.u1)
    annotation (Line(points={{60,-28},{60,-38}}, color={255,0,255}));
  connect(lesThrVal1.y, andVal2.u2) annotation (Line(points={{58,30},{46,30},{46,
          -32},{52,-32},{52,-38}},
                          color={255,0,255}));
  connect(andVal2.y, swiVal2.u2) annotation (Line(points={{60,-62},{60,-70},{70,
          -70},{70,-118}}, color={255,0,255}));
  annotation (Diagram(coordinateSystem(extent={{-100,-160},{100,100}})), Icon(
        coordinateSystem(extent={{-100,-100},{100,100}})),
    Documentation(revisions="<html>
<ul>
<li>
February 18, 2022 by Hongxiang Fu:<br/>
First implementation. This is for
<a href=\"https://github.com/lbl-srg/modelica-buildings/issues/2859\">#2859</a>.
</li>
</ul>
</html>", info="<html>
<p>
This is a control block for the secondary pump-valve group in
<a href=\"Modelica://Buildings.Fluid.Storage.Plant.ChillerAndTank\">
Buildings.Fluid.Storage.Plant.ChillerAndTank</a>.
This block is conditionally enabled when the plant is configured to allow
remotely charging the tank.
</p>
</html>"));
end ReversiblePumpValveControl;