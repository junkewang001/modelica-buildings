within ControlSequence;
package DXCoilStage

  model DXCoilStage
    "Validation models of determining heating and coooling loop signal"
    Buildings.Controls.OBC.ASHRAE.G36.ThermalZones.ControlLoops conLoo
      "Heating and cooling loop"
      annotation (Placement(transformation(extent={{-48,4},{-34,20}})));
    Buildings.Controls.OBC.CDL.Continuous.Sources.Constant TZonCooSetOcc(final k=
          298.15)
      "Occupied cooling setpoint"
      annotation (Placement(transformation(extent={{-88,26},{-76,38}})));
    Buildings.Controls.OBC.CDL.Continuous.Sources.Constant TZonHeaSetOcc(
      final k=293.15)
      "Occupied heating setpoint"
      annotation (Placement(transformation(extent={{-88,-16},{-76,-4}})));
    Buildings.Controls.OBC.CDL.Continuous.Sources.Sine zonTem(
      final amplitude=8,
      final freqHz=1/7200,
      final offset=273.15 + 18) "Zone temperature"
      annotation (Placement(transformation(extent={{-88,6},{-76,18}})));

    Buildings.Controls.OBC.CDL.Continuous.Sources.Sine TOut(
      final amplitude=5,
      final offset=18 + 273.15,
      final freqHz=1/3600) "Outdoor air temperature"
      annotation (Placement(transformation(extent={{-88,48},{-76,60}})));
    Buildings.Controls.OBC.CDL.Continuous.Sources.Ramp supFanSpe(
      final duration=3600,
      final height=0.7,
      final offset=0.1) "Supply fan speed"
      annotation (Placement(transformation(extent={{-88,-38},{-76,-26}})));
  equation
    connect(TZonCooSetOcc.y, conLoo.TCooSet) annotation (Line(points={{-74.8,32},
            {-60,32},{-60,16.8},{-49.4,16.8}},
                                   color={0,0,127}));
    connect(TZonHeaSetOcc.y, conLoo.THeaSet) annotation (Line(points={{-74.8,
            -10},{-60,-10},{-60,7.2},{-49.4,7.2}},
                                      color={0,0,127}));
    connect(zonTem.y, conLoo.TZon)
      annotation (Line(points={{-74.8,12},{-49.4,12}},
                                                color={0,0,127}));

  annotation (experiment(StopTime=7200, Tolerance=1e-06),
    __Dymola_Commands(file="modelica://Buildings/Resources/Scripts/Dymola/Controls/OBC/ASHRAE/G36/ThermalZones/Validation/ControlLoops.mos"
      "Simulate and plot"),
      Icon(coordinateSystem(preserveAspectRatio=false), graphics={Ellipse(
            lineColor={75,138,73},
            fillColor={255,255,255},
            fillPattern=FillPattern.Solid,
            extent={{-100,-100},{100,100}}), Polygon(
            lineColor={0,0,255},
            fillColor={75,138,73},
            pattern=LinePattern.None,
            fillPattern=FillPattern.Solid,
            points={{-36,58},{64,-2},{-36,-62},{-36,58}})}),
      Diagram(coordinateSystem(preserveAspectRatio=false)),
  Documentation(info="<html>
<p>
This example validates
<a href=\"modelica://Buildings.Controls.OBC.ASHRAE.G36.ThermalZones.ControlLoops\">
Buildings.Controls.OBC.ASHRAE.G36.ThermalZones.ControlLoops</a>.
</p>
</html>",   revisions="<html>
<ul>
<li>
February 2, 2022, by Jianjun Hu:<br/>
First implementation.
</li>
</ul>
</html>"));
  end DXCoilStage;

  block DXCoilCom "RTU coil staging control loop"

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

    Buildings.Controls.OBC.CDL.Interfaces.BooleanInput uCoiAva[nCoi]
      "DX coil availability status vector" annotation (Placement(transformation(
            extent={{-124,8},{-100,32}}), iconTransformation(extent={{-140,0},{
              -100,40}})));
    Buildings.Controls.OBC.CDL.Interfaces.RealOutput yComSpe[nCoi](
      final min=0,
      final max=1,
      final unit="1") "Compressor commanded speed" annotation (Placement(
          transformation(extent={{100,28},{124,52}}), iconTransformation(extent=
             {{100,-20},{140,20}})));

    Buildings.Controls.OBC.CDL.Interfaces.RealInput uCooCoi(unit="1")
      "Cooling coil valve position" annotation (Placement(transformation(extent=
             {{-124,-72},{-100,-48}}), iconTransformation(extent={{-140,40},{
              -100,80}})));
    Buildings.Controls.OBC.CDL.Interfaces.BooleanInput uStaChaProEnd
      "Signal indicating end of stage change process"
      annotation (Placement(transformation(extent={{-124,-32},{-100,-8}}),
        iconTransformation(extent={{-140,150},{-100,190}},
          rotation=0,
          origin={0,-230})));
    Buildings.Controls.OBC.CDL.Interfaces.BooleanInput uComPro
      "Compressor Proven on signal" annotation (Placement(transformation(extent=
             {{-124,48},{-100,72}}), iconTransformation(extent={{-140,-40},{
              -100,0}})));
    Buildings.Controls.OBC.CDL.Interfaces.IntegerOutput ySta(final min=0,
        final max=nSta) "DX coil stage integer setpoint"
      annotation (Placement(transformation(extent={{100,-12},{124,12}}),
        iconTransformation(extent={{100,40},{140,80}})));
    Buildings.Controls.OBC.CDL.Interfaces.BooleanOutput yCoi[nCoi]
      "DX coil status setpoint vector for the current coil stage setpoint"
      annotation (Placement(transformation(extent={{100,-72},{124,-48}}),
          iconTransformation(extent={{100,-80},{140,-40}})));
    Buildings.Controls.OBC.ASHRAE.PrimarySystem.BoilerPlant.Staging.SetPoints.Subsequences.Status
      sta annotation (Placement(transformation(extent={{-26,-28},{-6,-8}})));
    Buildings.Controls.OBC.ASHRAE.PrimarySystem.BoilerPlant.Staging.SetPoints.Subsequences.Capacities
      cap annotation (Placement(transformation(extent={{-42,16},{-22,36}})));
    Buildings.Controls.OBC.ASHRAE.PrimarySystem.BoilerPlant.Staging.SetPoints.Subsequences.Change
      cha annotation (Placement(transformation(extent={{16,0},{36,20}})));
    Buildings.Controls.OBC.ASHRAE.PrimarySystem.BoilerPlant.Staging.SetPoints.Subsequences.BoilerIndices
      boiInd annotation (Placement(transformation(extent={{58,-70},{78,-50}})));
    Buildings.Controls.OBC.ASHRAE.PrimarySystem.BoilerPlant.Staging.SetPoints.Subsequences.Configurator
      conf annotation (Placement(transformation(extent={{-82,10},{-62,30}})));
  equation

    connect(boiInd.yBoi, yCoi)
      annotation (Line(points={{80,-60},{112,-60}}, color={255,0,255}));
    connect(cha.ySta, ySta) annotation (Line(points={{38,16},{70,16},{70,0},{
            112,0}}, color={255,127,0}));
    connect(cha.ySta, boiInd.u) annotation (Line(points={{38,16},{46,16},{46,
            -60},{56,-60}}, color={255,127,0}));
    connect(uCoiAva, conf.uBoiAva)
      annotation (Line(points={{-112,20},{-84,20}}, color={255,0,255}));
    connect(sta.uAva, conf.yAva) annotation (Line(points={{-28,-24},{-60,-24},{
            -60,12}}, color={255,0,255}));
    connect(cha.uStaAva, conf.yAva) annotation (Line(points={{14,18},{-12,18},{
            -12,8},{-52,8},{-52,12},{-60,12}}, color={255,0,255}));
    connect(sta.yAvaUp, cha.uAvaUp)
      annotation (Line(points={{-4,-11},{-4,12},{14,12}}, color={255,127,0}));
    connect(sta.yAvaDow, cha.uAvaDow) annotation (Line(points={{-4,-14},{8,-14},
            {8,8},{14,8}}, color={255,127,0}));
  annotation (
    defaultComponentName = "supSig",
    Icon(coordinateSystem(preserveAspectRatio=false), graphics={
          Text(
            extent={{-98,-12},{-62,-28}},
            textColor={0,0,127},
            pattern=LinePattern.Dash,
            textString="uComPro"),
          Text(
            extent={{60,8},{98,-4}},
            textColor={0,0,127},
            pattern=LinePattern.Dash,
            visible=have_heaCoi,
            textString="yComSpe[nCoi]"),
          Text(
            extent={{74,66},{96,54}},
            textColor={0,0,127},
            pattern=LinePattern.Dash,
            textString="ySta"),
          Text(
            extent={{62,-50},{96,-64}},
            textColor={0,0,127},
            pattern=LinePattern.Dash,
            textString="yCoi[nCoi]"),
          Text(
            extent={{-98,68},{-58,54}},
            textColor={0,0,127},
            pattern=LinePattern.Dash,
            textString="uCooCoi"),
          Text(
            extent={{-114,136},{106,98}},
            textColor={0,0,255},
            textString="%name"),
          Text(
            extent={{-98,28},{-58,14}},
            textColor={0,0,127},
            pattern=LinePattern.Dash,
            textString="uCoiAva[nCoi]"),
          Text(
            extent={{-98,-52},{-58,-66}},
            textColor={0,0,127},
            pattern=LinePattern.Dash,
            textString="uStaChaProEnd"),
          Rectangle(extent={{-100,-100},{100,100}}, lineColor={0,0,127})}),
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
</html>"),
      Diagram(coordinateSystem(preserveAspectRatio=false)));
  end DXCoilCom;

  block Configurator "Configures DX coil staging"

    parameter Integer nSta = 5
      "Number of boiler stages";

    parameter Integer nBoi = 3
      "Number of boilers";

    parameter Integer boiTyp[nBoi]={
      Buildings.Controls.OBC.ASHRAE.PrimarySystem.BoilerPlant.Types.BoilerTypes.condensingBoiler,
      Buildings.Controls.OBC.ASHRAE.PrimarySystem.BoilerPlant.Types.BoilerTypes.nonCondensingBoiler,
      Buildings.Controls.OBC.ASHRAE.PrimarySystem.BoilerPlant.Types.BoilerTypes.nonCondensingBoiler}
      "Boiler type. Recommended staging order: 1. condensing boilers, 2. non-codensing boilers";

    parameter Integer staMat[nSta, nBoi] = {{1,0,0},{0,1,0},{1,1,0},{0,1,1},{1,1,1}}
      "Staging matrix with stage as row index and boiler as column index";

    parameter Real boiDesCap[nBoi](
      final unit="W",
      displayUnit="W",
      final quantity="Power")
      "Design boiler capacities vector";

    parameter Real boiFirMin[nBoi](
      final unit="1",
      displayUnit="1")
      "Boiler minimum firing ratios vector";

    Buildings.Controls.OBC.CDL.Interfaces.BooleanInput uCoiAva[nCoi]
      "Coil availability status vector"
      annotation (Placement(transformation(extent={{-260,-130},{-220,-90}}),
        iconTransformation(extent={{-140,-20},{-100,20}})));

    Buildings.Controls.OBC.CDL.Interfaces.BooleanOutput yAva[nSta]
      "Stage availability status vector"
      annotation (Placement(transformation(extent={{220,-100},{260,-60}}),
        iconTransformation(extent={{100,-100},{140,-60}})));

    Buildings.Controls.OBC.CDL.Interfaces.RealOutput yCapDes[nSta](
      final unit=fill("W", nSta),
      final quantity=fill("Power", nSta))
      "Stage design capacities vector"
      annotation (Placement(transformation(extent={{220,0},{260,40}}),
        iconTransformation(extent={{100,60},{140,100}})));

    Buildings.Controls.OBC.CDL.Interfaces.RealOutput yCapMin[nSta](
      final unit=fill("W", nSta),
      final quantity=fill("Power", nSta))
      "Minimum stage capacities vector"
      annotation (Placement(transformation(extent={{220,-40},{260,0}}),
        iconTransformation(extent={{100,20},{140,60}})));

    Buildings.Controls.OBC.CDL.Utilities.Assert assMes1(
      final message="The boilers must be tagged in order of design capacity if unequally sized")
      "Asserts whether boilers are tagged in ascending order with regards to capacity"
      annotation (Placement(transformation(extent={{60,150},{80,170}})));

    Buildings.Controls.OBC.CDL.Continuous.Subtract sub1[nBoi]
      "Subtracts signals"
      annotation (Placement(transformation(extent={{-100,150},{-80,170}})));

    Buildings.Controls.OBC.CDL.Continuous.MultiMax multiMax(
      nin=nBoi)
      "Maximum value in a vector input"
      annotation (Placement(transformation(extent={{-60,150},{-40,170}})));

    Buildings.Controls.OBC.CDL.Continuous.Abs abs
      "Absolute values"
      annotation (Placement(transformation(extent={{-20,150},{0,170}})));

    Buildings.Controls.OBC.CDL.Continuous.LessThreshold lesThr1(
      final t=0.5)
      "Less threshold"
      annotation (Placement(transformation(extent={{20,150},{40,170}})));

  protected
    final parameter Integer boiTypMat[nSta, nBoi] = {boiTyp[i] for i in 1:nBoi, j in 1:nSta}
      "Boiler type array expanded to allow for element-wise multiplication with the
    staging matrix";

    final parameter Real boiFirMinVal[nSta, nBoi] = {boiFirMin[i] for i in 1:nBoi, j in 1:nSta}
      "Boiler minimum firing ratio array expanded for element-wise multiplication
    with the staging matrix";

    Buildings.Controls.OBC.CDL.Continuous.Sources.Constant coiDesCaps[nCoi](
        final k=coiDesCap) "Design coil capacities vector"
      annotation (Placement(transformation(extent={{-200,100},{-180,120}})));

    Buildings.Controls.OBC.CDL.Continuous.Sources.Constant boiFirMinMat[nSta,nBoi](
      final k=boiFirMinVal)
      "Boiler minimum firing ratios matrix"
      annotation (Placement(transformation(extent={{-200,60},{-180,80}})));

    Buildings.Controls.OBC.CDL.Continuous.MatrixGain staDesCaps(
      final K=staMat)
      "Matrix gain for design capacities"
      annotation (Placement(transformation(extent={{-140,100},{-120,120}})));

    Buildings.Controls.OBC.CDL.Continuous.MatrixGain sumNumCoi(
      final K=staMat) "Outputs the total coil count per stage vector"
      annotation (Placement(transformation(extent={{-140,-62},{-120,-42}})));

    Buildings.Controls.OBC.CDL.Continuous.MatrixGain sumNumAvaCoi(
      final K=staMat) "Outputs the available coil count per stage vector"
      annotation (Placement(transformation(extent={{-140,-120},{-120,-100}})));

    Buildings.Controls.OBC.CDL.Continuous.Sources.Constant oneVec[nCoi](final k=
         fill(1, nCoi)) "Mocks a case with all coils available"
      annotation (Placement(transformation(extent={{-200,-62},{-180,-42}})));

    Buildings.Controls.OBC.CDL.Conversions.BooleanToReal booToRea[nCoi]
      "Type converter"
      annotation (Placement(transformation(extent={{-200,-120},{-180,-100}})));

    Buildings.Controls.OBC.CDL.Continuous.Subtract sub2[nSta]
      "Subtracts count of available coils from the design count, at each stage"
      annotation (Placement(transformation(extent={{-80,-90},{-60,-70}})));

    Buildings.Controls.OBC.CDL.Continuous.LessThreshold lesThr[nSta](
      final t=fill(0.5, nSta))
      "Checks if the count of available coils in each stage equals the design count"
      annotation (Placement(transformation(extent={{-40,-90},{-20,-70}})));

    Buildings.Controls.OBC.CDL.Continuous.Sources.Constant boiStaMat[nSta,nBoi](
      final k=staMat)
      "Staging matrix"
      annotation (Placement(transformation(extent={{-200,18},{-180,38}})));

    Buildings.Controls.OBC.CDL.Continuous.Multiply pro1[nSta,nBoi]
      "Element-wise product"
      annotation (Placement(transformation(extent={{-160,40},{-140,60}})));

    Buildings.Controls.OBC.CDL.Continuous.MatrixMax matMax1(
      final rowMax=true,
      final nRow=nSta,
      final nCol=nBoi)
      "Find highest BFirMin in each stage"
      annotation (Placement(transformation(extent={{-120,40},{-100,60}})));

    Buildings.Controls.OBC.CDL.Continuous.Multiply pro2[nSta]
      "Product"
      annotation (Placement(transformation(extent={{-80,60},{-60,80}})));

    Buildings.Controls.OBC.CDL.Continuous.Sort sort1(
      final nin=nBoi)
      "Sort values"
      annotation (Placement(transformation(extent={{-140,160},{-120,180}})));

  equation
    connect(coiDesCaps.y, staDesCaps.u)
      annotation (Line(points={{-178,110},{-142,110}},
        color={0,0,127}));
    connect(uCoiAva, booToRea.u)
      annotation (Line(points={{-240,-110},{-202,-110}},
        color={255,0,255}));
    connect(booToRea.y,sumNumAvaCoi. u)
      annotation (Line(points={{-178,-110},{-142,-110}},
        color={0,0,127}));
    connect(sumNumCoi.y,sub2. u1)
      annotation (Line(points={{-118,-52},{-100,-52},{-100,-74},{-82,-74}},
        color={0,0,127}));
    connect(sumNumAvaCoi.y,sub2. u2)
      annotation (Line(points={{-118,-110},{-100.5,-110},{-100.5,-86},{-82,-86}},
        color={0,0,127}));
    connect(sub2.y,lesThr. u)
      annotation (Line(points={{-58,-80},{-42,-80}},
        color={0,0,127}));
    connect(lesThr.y, yAva)
      annotation (Line(points={{-18,-80},{240,-80}},
        color={255,0,255}));
    connect(staDesCaps.y,yCapDes)
      annotation (Line(points={{-118,110},{100,110},{100,20},{240,20}},
        color={0,0,127}));
    connect(oneVec.y,sumNumCoi. u)
      annotation (Line(points={{-178,-52},{-142,-52}},
        color={0,0,127}));
    connect(boiFirMinMat.y, pro1.u1)
      annotation (Line(points={{-178,70},{-170,70},{-170,56},{-162,56}},
        color={0,0,127}));
    connect(pro1.u2, boiStaMat.y)
      annotation (Line(points={{-162,44},{-172,44},{-172,28},{-178,28}},
        color={0,0,127}));
    connect(pro1.y, matMax1.u)
      annotation (Line(points={{-138,50},{-122,50}},
        color={0,0,127}));
    connect(matMax1.y, pro2.u2)
      annotation (Line(points={{-98,50},{-90,50},{-90,64},{-82,64}},
        color={0,0,127}));
    connect(staDesCaps.y, pro2.u1)
      annotation (Line(points={{-118,110},{-90,110},{-90,76},{-82,76}},
        color={0,0,127}));
    connect(pro2.y,yCapMin)
      annotation (Line(points={{-58,70},{80,70},{80,-20},{240,-20}},
        color={0,0,127}));

    connect(coiDesCaps.y,sub1. u2) annotation (Line(points={{-178,110},{-160,110},
            {-160,154},{-102,154}}, color={0,0,127}));
    connect(sort1.y,sub1. u1) annotation (Line(points={{-118,170},{-110,170},
            {-110,166},{-102,166}}, color={0,0,127}));
    connect(sub1.y, multiMax.u) annotation (Line(points={{-78,160},{-70,160},
            {-70,160},{-62,160}}, color={0,0,127}));
    connect(multiMax.y, abs.u)
      annotation (Line(points={{-38,160},{-22,160}}, color={0,0,127}));
    connect(abs.y, lesThr1.u)
      annotation (Line(points={{2,160},{18,160}}, color={0,0,127}));
    connect(lesThr1.y, assMes1.u)
      annotation (Line(points={{42,160},{58,160}}, color={255,0,255}));
    connect(sort1.u,coiDesCaps. y) annotation (Line(points={{-142,170},{-160,
            170},{-160,110},{-178,110}}, color={0,0,127}));
    annotation (defaultComponentName = "conf",
      Icon(graphics={
             Rectangle(extent={{-100,-100},{100,100}},
                       lineColor={0,0,127},
                       fillColor={255,255,255},
                       fillPattern=FillPattern.Solid),
                  Text(extent={{-120,146},{100,108}},
                       textColor={0,0,255},
                       textString="%name")}),
      Diagram(coordinateSystem(preserveAspectRatio=false,
        extent={{-220,-200},{220,200}})),
      Documentation(info="<html>
      <p>
      This subsequence is not directly specified in 1711 as it provides
      a side calculation pertaining to generalization of the staging 
      sequences for any number of boilers and stages provided by the 
      user.
      </p>
      <p>
      Given the staging matrix input parameter <code>staMat</code> the staging
      configurator calculates:
      </p>
      <ul>
      <li>
      Stage availability vector <code>yAva</code> from the boiler availability
      <code>uBoiAva</code> input vector according to RP-1711 March 2020 Draft
      section 5.3.3.9.
      </li>
      <li>
      Design stage capacity vector <code>yDesCap</code> from the design boiler
      capacity vector input parameter <code>boiDesCap</code>.
      </li>
      <li>
      Minimum stage capacity vector <code>yMinCap</code> from the boiler minimum
      firing rate input parameter <code>boiMinCap</code> according to section
      5.3.3.8, 1711 March 2020 Draft.
      </li>
      <li>
      Stage type vector <code>yTyp</code> from the boiler type vector input
      parameter <code>boiTyp</code>. Boiler types are defined in
      <a href=\"modelica://Buildings.Controls.OBC.ASHRAE.PrimarySystem.BoilerPlant.Types.BoilerTypes\">
      Buildings.Controls.OBC.ASHRAE.PrimarySystem.BoilerPlant.Types.BoilerTypes</a>.<br/>
      Stage type is based on the boiler types in that stage, and is classified
      as:
      <ol>
      <li>
      non-condensing, if any of the boilers in that stage are non-condensing boilers.
      </li>
      <li>
      condensing, if all the boilers in that stage are condensing boilers.
      </li>
      </ol>
      This stage type is used to determine the stage up and down conditions to apply.
      </li>
      </ul>
      </html>",
        revisions="<html>
      <ul>
      <li>
      May 20, 2020, by Karthik Devaprasad:<br/>
      First implementation.
      </li>
      </ul>
      </html>"));
  end Configurator;
end DXCoilStage;
