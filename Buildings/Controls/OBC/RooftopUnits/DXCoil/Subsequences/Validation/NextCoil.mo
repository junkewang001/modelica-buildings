within Buildings.Controls.OBC.RooftopUnits.DXCoil.Subsequences.Validation;
model NextCoil
  CDL.Integers.Sources.Constant conInt[4](k=coiInd)
    annotation (Placement(transformation(extent={{-90,70},{-70,90}})));
  CDL.Logical.Sources.Constant con(k=true)
    annotation (Placement(transformation(extent={{-90,10},{-70,30}})));
  CDL.Logical.Sources.SampleTrigger samTri(period=60, shift=10)
    annotation (Placement(transformation(extent={{-40,20},{-20,40}})));

  CDL.Logical.TrueFalseHold truFalHol1(trueHoldDuration=10, falseHoldDuration=0)
    annotation (Placement(transformation(extent={{20,30},{40,50}})));
  NextCoil_v2 nextCoil_v2_1(nCoi=nCoi)
    annotation (Placement(transformation(extent={{20,0},{40,20}})));
  CDL.Logical.Pre pre1
                     [4]
    annotation (Placement(transformation(extent={{94,0},{114,20}})));
  ChangeStatus chaSta1(nCoi=4)
    annotation (Placement(transformation(extent={{60,0},{80,20}})));
  CDL.Logical.TrueDelay truDel1(delayTime=10)
    annotation (Placement(transformation(extent={{-40,-20},{-20,0}})));
  CDL.Logical.TrueDelay truDel2(delayTime=10)
    annotation (Placement(transformation(extent={{-90,-20},{-70,0}})));
  NextCoil_v2 nextCoil_v2_2(nCoi=nCoi)
    annotation (Placement(transformation(extent={{20,-80},{40,-60}})));
  ChangeStatus chaSta2(nCoi=4)
    annotation (Placement(transformation(extent={{60,-80},{80,-60}})));
  CDL.Logical.Pre pre2
                     [4]
    annotation (Placement(transformation(extent={{94,-80},{114,-60}})));
  CDL.Logical.TrueDelay truDel3(delayTime=10)
    annotation (Placement(transformation(extent={{-40,-90},{-20,-70}})));
  CDL.Logical.TrueDelay truDel4(delayTime=10)
    annotation (Placement(transformation(extent={{-90,-70},{-70,-50}})));
protected
  parameter Integer nCoi=4;
  parameter Integer coiInd[nCoi]={i for i in 1:nCoi}
    "DX coil index, {1,2,...,n}";
equation
  connect(samTri.y, truFalHol1.u) annotation (Line(points={{-18,30},{4,30},{4,
          40},{18,40}},   color={255,0,255}));
  connect(conInt.y, nextCoil_v2_1.uCoiSeq) annotation (Line(points={{-68,80},{
          10,80},{10,16},{18,16}},        color={255,127,0}));
  connect(samTri.y, nextCoil_v2_1.uStaUp) annotation (Line(points={{-18,30},{4,
          30},{4,12},{18,12}},      color={255,0,255}));
  connect(chaSta1.yDXCoi, pre1.u)
    annotation (Line(points={{82,10},{92,10}},       color={255,0,255}));
  connect(pre1.y, nextCoil_v2_1.uDXCoi) annotation (Line(points={{116,10},{130,
          10},{130,-12},{0,-12},{0,4},{18,4}},           color={255,0,255}));
  connect(con.y, nextCoil_v2_1.uDXCoiAva[1]) annotation (Line(points={{-68,20},
          {-60,20},{-60,8},{18,8}},      color={255,0,255}));
  connect(truDel1.y, nextCoil_v2_1.uDXCoiAva[2]) annotation (Line(points={{-18,-10},
          {-4,-10},{-4,6},{18,6},{18,8}},           color={255,0,255}));
  connect(nextCoil_v2_1.yStaUp, chaSta1.uNexDXCoiSta) annotation (Line(points={{42,12},
          {50,12},{50,18},{58,18}},                color={255,0,255}));
  connect(nextCoil_v2_1.yNexCoi, chaSta1.uNexDXCoi) annotation (Line(points={{42,8},{
          50,8},{50,6},{58,6}},                 color={255,127,0}));
  connect(nextCoil_v2_1.yNexCoi, chaSta1.uLasDXCoi) annotation (Line(points={{42,8},{
          50,8},{50,2},{58,2}},                 color={255,127,0}));
  connect(pre1.y, chaSta1.uDXCoil) annotation (Line(points={{116,10},{130,10},{
          130,-12},{54,-12},{54,10},{58,10}},        color={255,0,255}));
  connect(con.y, chaSta1.uLasDXCoiSta) annotation (Line(points={{-68,20},{-60,
          20},{-60,-40},{52,-40},{52,14},{58,14}},   color={255,0,255}));
  connect(truDel2.u, truDel1.u) annotation (Line(points={{-92,-10},{-92,-30},{
          -46,-30},{-46,-10},{-42,-10}},    color={255,0,255}));
  connect(truDel2.y, nextCoil_v2_1.uDXCoiAva[3]) annotation (Line(points={{-68,-10},
          {-56,-10},{-56,10},{18,10},{18,8}},         color={255,0,255}));
  connect(con.y, nextCoil_v2_1.uDXCoiAva[4]) annotation (Line(points={{-68,20},
          {-60,20},{-60,8},{18,8}},      color={255,0,255}));
  connect(pre1[4].y, truDel2.u) annotation (Line(points={{116,10},{130,10},{130,
          -34},{-96,-34},{-96,-10},{-92,-10}},         color={255,0,255}));
  connect(chaSta2.yDXCoi,pre2. u)
    annotation (Line(points={{82,-70},{92,-70}},     color={255,0,255}));
  connect(nextCoil_v2_2.yStaUp, chaSta2.uNexDXCoiSta) annotation (Line(points={
          {42,-68},{50,-68},{50,-62},{58,-62}}, color={255,0,255}));
  connect(nextCoil_v2_2.yNexCoi, chaSta2.uNexDXCoi) annotation (Line(points={{
          42,-72},{50,-72},{50,-74},{58,-74}}, color={255,127,0}));
  connect(pre2.y, chaSta2.uDXCoil) annotation (Line(points={{116,-70},{120,-70},
          {120,-90},{54,-90},{54,-70},{58,-70}}, color={255,0,255}));
  connect(nextCoil_v2_2.yNexCoi, chaSta2.uLasDXCoi) annotation (Line(points={{
          42,-72},{50,-72},{50,-78},{58,-78}}, color={255,127,0}));
  connect(con.y, chaSta2.uLasDXCoiSta) annotation (Line(points={{-68,20},{-60,
          20},{-60,-40},{52,-40},{52,-66},{58,-66}}, color={255,0,255}));
  connect(pre2[3].y, truDel3.u) annotation (Line(points={{116,-70},{120,-70},{
          120,-100},{-50,-100},{-50,-80},{-42,-80}}, color={255,0,255}));
  connect(conInt.y, nextCoil_v2_2.uCoiSeq) annotation (Line(points={{-68,80},{
          10,80},{10,-64},{18,-64}}, color={255,127,0}));
  connect(truDel3.y, nextCoil_v2_2.uDXCoiAva[2]) annotation (Line(points={{-18,
          -80},{-14,-80},{-14,-72},{18,-72}}, color={255,0,255}));
  connect(pre2[2].y, truDel4.u) annotation (Line(points={{116,-70},{120,-70},{
          120,-100},{-100,-100},{-100,-60},{-92,-60}}, color={255,0,255}));
  connect(truDel4.y, nextCoil_v2_2.uDXCoiAva[4]) annotation (Line(points={{-68,
          -60},{-10,-60},{-10,-76},{14,-76},{14,-72},{18,-72}}, color={255,0,
          255}));
  connect(con.y, nextCoil_v2_2.uDXCoiAva[1]) annotation (Line(points={{-68,20},
          {-60,20},{-60,-40},{0,-40},{0,-74},{18,-74},{18,-72}}, color={255,0,
          255}));
  connect(con.y, nextCoil_v2_2.uDXCoiAva[3]) annotation (Line(points={{-68,20},
          {-60,20},{-60,-40},{0,-40},{0,-70},{14,-70},{14,-72},{18,-72}}, color
        ={255,0,255}));
  connect(pre2.y, nextCoil_v2_2.uDXCoi) annotation (Line(points={{116,-70},{120,
          -70},{120,-90},{16,-90},{16,-76},{18,-76}}, color={255,0,255}));
  connect(samTri.y, nextCoil_v2_2.uStaUp) annotation (Line(points={{-18,30},{4,
          30},{4,-68},{18,-68}}, color={255,0,255}));
  annotation (Icon(coordinateSystem(preserveAspectRatio=false, extent={{-100,-100},
            {100,100}}),                                        graphics={
          Ellipse(lineColor = {75,138,73},
              fillColor={255,255,255},
              fillPattern = FillPattern.Solid,
              extent={{-100,-100},{100,100}}),
          Polygon(lineColor = {0,0,255},
              fillColor = {75,138,73},
              pattern = LinePattern.None,
              fillPattern = FillPattern.Solid,
              points={{-36,60},{64,0},{-36,-60},{-36,60}})}), Diagram(
        coordinateSystem(preserveAspectRatio=false, extent={{-120,-160},{140,100}})),
    experiment(
      StopTime=240,
      Interval=1,
      __Dymola_Algorithm="Cvode"));
end NextCoil;
