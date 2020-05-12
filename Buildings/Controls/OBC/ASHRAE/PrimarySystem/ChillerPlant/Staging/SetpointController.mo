within Buildings.Controls.OBC.ASHRAE.PrimarySystem.ChillerPlant.Staging;
block SetpointController
  "Calculates the chiller stage status setpoint signal"

  parameter Boolean have_WSE = false
    "true = plant has a WSE, false = plant does not have WSE"
    annotation (Dialog(tab="General", group="Plant configuration parameters"));

  parameter Boolean serChi = false
    "true = series chillers plant; false = parallel chillers plant"
    annotation (Dialog(tab="General", group="Plant configuration parameters"));

  parameter Boolean anyVsdCen = false
    "Plant contains at least one variable speed centrifugal chiller"
    annotation (Dialog(tab="General", group="Plant configuration parameters"));

  parameter Integer nChi = 2
    "Number of chillers"
    annotation (Dialog(tab="General", group="Chiller configuration parameters"));

  parameter Modelica.SIunits.Power chiDesCap[nChi]
    "Design chiller capacities vector"
    annotation (Dialog(tab="General", group="Chiller configuration parameters"));

  parameter Modelica.SIunits.Power chiMinCap[nChi]
    "Chiller minimum cycling loads vector"
    annotation (Dialog(tab="General", group="Chiller configuration parameters"));

  parameter Integer chiTyp[nChi]={
    Buildings.Controls.OBC.ASHRAE.PrimarySystem.ChillerPlant.Types.ChillerAndStageTypes.positiveDisplacement,
    Buildings.Controls.OBC.ASHRAE.PrimarySystem.ChillerPlant.Types.ChillerAndStageTypes.variableSpeedCentrifugal}
    "Chiller type. Recommended staging order: positive displacement, variable speed centrifugal, constant speed centrifugal"
    annotation (Dialog(tab="General", group="Chiller configuration parameters"));

  parameter Integer nSta = 3
    "Number of chiller stages"
    annotation (Dialog(tab="General", group="Chiller configuration parameters"));

  parameter Integer staMat[nSta, nChi] = {{1,0},{0,1},{1,1}}
    "Staging matrix with stage as row index and chiller as column index"
    annotation (Dialog(tab="General", group="Chiller configuration parameters"));

  parameter Modelica.SIunits.Time avePer = 300
    "Time period for the capacity requirement rolling average"
    annotation (Dialog(tab="Time parameters", group="Hold and delay parameters"));

  parameter Modelica.SIunits.Time delayStaCha = 900
    "Hold period for each stage change"
    annotation (Dialog(tab="Time parameters", group="Hold and delay parameters"));

  parameter Modelica.SIunits.Time parLoaRatDelay = 900
    "Enable delay for operating and staging part load ratio condition"
    annotation (Dialog(tab="Time parameters", group="Hold and delay parameters"));

  parameter Modelica.SIunits.Time faiSafTruDelay = 900
    "Enable delay for failsafe condition"
    annotation (Dialog(tab="Time parameters", group="Hold and delay parameters"));

  parameter Modelica.SIunits.Time effConTruDelay = 900
    "Enable delay for efficiency condition"
    annotation (Dialog(tab="Time parameters", group="Hold and delay parameters"));

  parameter Modelica.SIunits.Time shortTDelay = 600
    "Short enable delay for staging from zero to first available stage up"
    annotation(Evaluate=true, Dialog(enable=have_WSE, tab="Time parameters", group="Hold and delay parameters"));

  parameter Modelica.SIunits.Time longTDelay = 1200
    "Long enable delay for staging from zero to first available stage up"
    annotation(Evaluate=true, Dialog(enable=have_WSE, tab="Time parameters", group="Hold and delay parameters"));

  parameter Real posDisMult(
    final unit = "1",
    final min = 0,
    final max = 1)=0.8
    "Positive displacement chiller type staging multiplier"
    annotation (Dialog(tab="Conditionals", group="Staging part load ratio parameters"));

  parameter Real conSpeCenMult(
    final unit = "1",
    final min = 0,
    final max = 1)=0.9
    "Constant speed centrifugal chiller type staging multiplier"
    annotation (Dialog(tab="Conditionals", group="Staging part load ratio parameters"));

  parameter Real anyOutOfScoMult(
    final unit = "1",
    final min = 0,
    final max = 1)=0.9
    "Outside of G36 recommended staging order chiller type SPLR multiplier"
    annotation(Evaluate=true, __cdl(ValueInReference=False), Dialog(tab="Conditionals", group="Staging part load ratio parameters"));

  parameter Real varSpeStaMin(
    final unit = "1",
    final min = 0.1,
    final max = 1)=0.45
    "Minimum stage up or down part load ratio for variable speed centrifugal stage types"
    annotation(Evaluate=true, Dialog(enable=anyVsdCen, tab="Conditionals", group="Staging part load ratio parameters"));

  parameter Real varSpeStaMax(
    final unit = "1",
    final min = varSpeStaMin,
    final max = 1)=0.9
    "Maximum stage up or down part load ratio for variable speed centrifugal stage types"
    annotation(Evaluate=true, Dialog(enable=anyVsdCen, tab="Conditionals", group="Staging part load ratio parameters"));

  parameter Modelica.SIunits.TemperatureDifference smallTDif = 1
    "Offset between the chilled water supply temperature and its setpoint for the long condition"
    annotation(Evaluate=true, Dialog(enable=have_WSE, tab="Conditionals", group="Value comparison parameters"));

  parameter Modelica.SIunits.TemperatureDifference largeTDif = 2
    "Offset between the chilled water supply temperature and its setpoint for the short condition"
    annotation(Evaluate=true, Dialog(enable=have_WSE, tab="Conditionals", group="Value comparison parameters"));

  parameter Modelica.SIunits.TemperatureDifference faiSafTDif = 1
    "Offset between the chilled water supply temperature and its setpoint for the failsafe condition"
    annotation (Dialog(tab="Conditionals", group="Value comparison parameters"));

  parameter Modelica.SIunits.PressureDifference dpDif = 2 * 6895
    "Offset between the chilled water pump diferential static pressure and its setpoint"
    annotation (Dialog(tab="Conditionals", group="Value comparison parameters"));

  parameter Modelica.SIunits.TemperatureDifference TDif = 1
    "Offset between the chilled water supply temperature and its setpoint for staging down to WSE only"
    annotation (Dialog(tab="Conditionals", group="Value comparison parameters"));

  parameter Modelica.SIunits.TemperatureDifference TDifHys = 1
    "Hysteresis deadband for temperature"
    annotation (Dialog(tab="Conditionals", group="Value comparison parameters"));

  parameter Modelica.SIunits.PressureDifference faiSafDpDif = 2 * 6895
    "Offset between the chilled water differential pressure and its setpoint"
    annotation (Dialog(tab="Conditionals", group="Value comparison parameters"));

  parameter Modelica.SIunits.PressureDifference dpDifHys = 0.5 * 6895
    "Pressure difference hysteresis deadband"
    annotation (Dialog(tab="Conditionals", group="Value comparison parameters"));

  parameter Real effConSigDif = 0.05
    "Signal hysteresis deadband"
    annotation (Dialog(tab="Conditionals", group="Value comparison parameters"));

  Buildings.Controls.OBC.CDL.Interfaces.BooleanInput uPla "Plant enable signal"
    annotation (Placement(
        transformation(extent={{-440,-120},{-400,-80}}),  iconTransformation(
          extent={{-140,-210},{-100,-170}})));

  Buildings.Controls.OBC.CDL.Interfaces.BooleanInput chaPro "Stage change process status signal"
    annotation (Placement(transformation(extent={{-440,-160},{-400,-120}}),
        iconTransformation(extent={{-140,-170},{-100,-130}})));

  Buildings.Controls.OBC.CDL.Interfaces.BooleanInput uWseSta if have_WSE
    "WSE status"
    annotation (Placement(transformation(extent={{-442,-280},{-402,-240}}),
        iconTransformation(extent={{-140,-150},{-100,-110}})));

  Buildings.Controls.OBC.CDL.Interfaces.BooleanInput uChiAva[nChi]
    "Chiller availability status vector"
    annotation (Placement(transformation(extent={{-442,-220},{-402,-180}}),
        iconTransformation(extent={{-140,-190},{-100,-150}})));

  Buildings.Controls.OBC.CDL.Interfaces.IntegerInput u(
    final min=0,
    final max=nSta) "Chiller stage"
    annotation (Placement(transformation(extent={{-440,-80},{-400,-40}}),
        iconTransformation(extent={{-140,-110},{-100,-70}})));

  Buildings.Controls.OBC.CDL.Interfaces.RealInput TWsePre(
    final unit="1") if have_WSE
    "Predicted WSE outlet temperature"
    annotation (Placement(transformation(extent={{-442,100},{-402,140}}),
       iconTransformation(extent={{-140,-70},{-100,-30}})));

  Buildings.Controls.OBC.CDL.Interfaces.RealInput uTowFanSpeMax if have_WSE
    "Maximum cooling tower fan speed"
    annotation (Placement(transformation(extent={{-442,70},{-402,110}}),
        iconTransformation(extent={{-140,-20},{-100,20}})));

  Buildings.Controls.OBC.CDL.Interfaces.RealInput uLifMin(
    final unit="K",
    final quantity="ThermodynamicTemperature") if anyVsdCen
    "Minimum chiller lift"
    annotation (Placement(transformation(extent={{-442,-30},{-402,10}}),
        iconTransformation(extent={{-140,60},{-100,100}})));

  Buildings.Controls.OBC.CDL.Interfaces.RealInput uLif(
    final unit="K",
    final quantity="ThermodynamicTemperature") if anyVsdCen
    "Chiller lift"
    annotation (Placement(transformation(extent={{-442,30},{-402,70}}),
        iconTransformation(extent={{-140,100},{-100,140}})));

  Buildings.Controls.OBC.CDL.Interfaces.RealInput uLifMax(
    final unit="K",
    final quantity="ThermodynamicTemperature") if anyVsdCen
    "Maximum chiller lift"
    annotation (Placement(transformation(extent={{-442,0},{-402,40}}),
        iconTransformation(extent={{-140,80},{-100,120}})));

  Buildings.Controls.OBC.CDL.Interfaces.RealInput TChiWatSupSet(
    final unit="K",
    final quantity="ThermodynamicTemperature")
    "Chilled water supply temperature setpoint"
    annotation (Placement(transformation(extent={{-442,350},{-402,390}}),
        iconTransformation(extent={{-140,150},{-100,190}})));

  Buildings.Controls.OBC.CDL.Interfaces.RealInput TChiWatRet(
    final unit="K",
    final quantity="ThermodynamicTemperature")
    "Chilled water return temperature"
    annotation (Placement(transformation(extent={{-442,280},{-402,320}}),
        iconTransformation(extent={{-140,-50},{-100,-10}})));

  Buildings.Controls.OBC.CDL.Interfaces.RealInput VChiWat_flow(
    final quantity="VolumeFlowRate",
    final unit="m3/s") "Measured chilled water flow rate"
    annotation (Placement(transformation(extent={{-442,250},{-402,290}}),
        iconTransformation(extent={{-140,-90},{-100,-50}})));

  Buildings.Controls.OBC.CDL.Interfaces.RealInput dpChiWatPumSet(
    final unit="Pa",
    final quantity="PressureDifference") if not serChi
    "Chilled water pump differential static pressure setpoint"
    annotation (Placement(transformation(extent={{-442,200},{-402,240}}),
      iconTransformation(extent={{-140,10},{-100,50}})));

  Buildings.Controls.OBC.CDL.Interfaces.RealInput dpChiWatPum(
    final unit="Pa",
    final quantity="PressureDifference") if not serChi
    "Chilled water pump differential static pressure"
    annotation (Placement(transformation(extent={{-442,170},{-402,210}}),
    iconTransformation(extent={{-140,30},{-100,70}})));

  Buildings.Controls.OBC.CDL.Interfaces.RealInput TChiWatSup(
    final unit="K",
    final quantity="ThermodynamicTemperature")
    "Chilled water return temperature"
    annotation (Placement(transformation(extent={{-442,320},{-402,360}}),
    iconTransformation(extent={{-140,130},{-100,170}})));

  Buildings.Controls.OBC.CDL.Interfaces.BooleanOutput yChiSet[nChi]
    "Chiller status setpoint vector for the current chiller stage setpoint"
    annotation (Placement(transformation(extent={{120,-280},{160,-240}}),
        iconTransformation(extent={{100,-90},{140,-50}})));

  Buildings.Controls.OBC.CDL.Interfaces.BooleanOutput yChaEdg
    "Chiller stage change edge signal" annotation (Placement(transformation(
          extent={{120,-200},{160,-160}}), iconTransformation(extent={{100,-20},
            {140,20}})));

  Buildings.Controls.OBC.CDL.Interfaces.IntegerOutput ySta(
    final min=0,
    final max=nSta)
    "Chiller stage integer setpoint"
    annotation (Placement(
        transformation(extent={{120,-140},{160,-100}}),iconTransformation(
          extent={{100,50},{140,90}})));

  Buildings.Controls.OBC.ASHRAE.PrimarySystem.ChillerPlant.Staging.Subsequences.Up staUp(
    final have_WSE=have_WSE,
    final serChi=serChi,
    final effConTruDelay=effConTruDelay,
    final faiSafTruDelay=faiSafTruDelay,
    final shortTDelay=shortTDelay,
    final longTDelay=longTDelay,
    final faiSafTDif=faiSafTDif,
    final TDifHys=TDifHys,
    final smallTDif=smallTDif,
    final largeTDif=largeTDif,
    final faiSafDpDif=faiSafDpDif,
    final dpDifHys=dpDifHys,
    final effConSigDif=effConSigDif) "Stage up conditions"
    annotation (Placement(transformation(extent={{-100,-120},{-80,-100}})));

  Buildings.Controls.OBC.ASHRAE.PrimarySystem.ChillerPlant.Staging.Subsequences.Down staDow(
    final have_WSE=have_WSE,
    final serChi=serChi,
    final parLoaRatDelay=parLoaRatDelay,
    final faiSafTruDelay=faiSafTruDelay,
    final faiSafTDif=faiSafTDif,
    final faiSafDpDif=faiSafDpDif,
    final TDif=TDif,
    final TDifHys=TDifHys) "Stage down conditions"
    annotation (Placement(transformation(extent={{-100,-240},{-80,-220}})));

  Buildings.Controls.OBC.ASHRAE.PrimarySystem.ChillerPlant.Staging.Subsequences.CapacityRequirement capReq(
    final avePer = avePer,
    final holPer = delayStaCha) "Capacity requirement"
    annotation (Placement(transformation(extent={{-322,300},{-302,320}})));

  Buildings.Controls.OBC.ASHRAE.PrimarySystem.ChillerPlant.Staging.Subsequences.Capacities cap(
    final nSta=nSta) "Design and minimum capacities for relevant chiller stages"
    annotation (Placement(transformation(extent={{-270,-180},{-250,-160}})));

  Subsequences.Initial iniSta(have_WSE=false)
    annotation (Placement(transformation(extent={{-80,100},{-60,120}})));
  CDL.Interfaces.RealInput                        uTunPar if have_WSE
    "Tuning parameter as at last plant disable"
    annotation (Placement(transformation(extent={{-440,130},{-400,170}}),
      iconTransformation(extent={{-140,40},{-100,80}})));
  CDL.Interfaces.RealInput                        TOutWet(final unit="K",
      final quantity="ThermodynamicTemperature") if
                                                  have_WSE
    "Outdoor air wet bulb temperature"
    annotation (Placement(transformation(extent={{-440,380},{-400,420}}),
      iconTransformation(extent={{-140,70},{-100,110}})));
//protected
  Buildings.Controls.OBC.ASHRAE.PrimarySystem.ChillerPlant.Staging.Subsequences.Configurator conf(
    final nSta = nSta,
    final nChi = nChi,
    final chiTyp = chiTyp,
    final chiDesCap = chiDesCap,
    final chiMinCap = chiMinCap,
    final staMat = staMat)
    "Configures chiller staging variables such as capacity and stage type vectors"
    annotation (Placement(transformation(extent={{-360,-180},{-340,-160}})));

  Buildings.Controls.OBC.ASHRAE.PrimarySystem.ChillerPlant.Staging.Subsequences.Status sta(
    final nSta=nSta,
    final nChi=nChi,
    final staMat=staMat) "First higher and lower available stage index, end stage boolean flags and chiller status setpoints"
    annotation (Placement(transformation(extent={{-320,-220},{-300,-200}})));

  Buildings.Controls.OBC.ASHRAE.PrimarySystem.ChillerPlant.Staging.Subsequences.PartLoadRatios PLRs(
    final anyVsdCen=anyVsdCen,
    final nSta=nSta,
    final posDisMult=posDisMult,
    final conSpeCenMult=conSpeCenMult,
    final anyOutOfScoMult=anyOutOfScoMult,
    final varSpeStaMin=varSpeStaMin,
    final varSpeStaMax=varSpeStaMax) "Operative and staging part load ratios"
    annotation (Placement(transformation(extent={{-182,-200},{-162,-180}})));

  Buildings.Controls.OBC.ASHRAE.PrimarySystem.ChillerPlant.Staging.Subsequences.Change cha(
    final nSta=nSta,
    final delayStaCha=delayStaCha) "Stage change assignment"
    annotation (Placement(transformation(extent={{-20,-180},{0,-160}})));

  Buildings.Controls.OBC.ASHRAE.PrimarySystem.ChillerPlant.Staging.Subsequences.ChillerIndices chiInd(
    nSta=nSta,
    nChi=nChi,
    staMat=staMat) "Calculates chiller status setpoint vector"
    annotation (Placement(transformation(extent={{40,-210},{60,-190}})));
equation
  connect(uChiAva, conf.uChiAva)
    annotation (Line(points={{-422,-200},{-382,-200},{-382,-170},{-362,-170}},
          color={255,0,255}));
  connect(conf.yAva, sta.uAva) annotation (Line(points={{-338,-178},{-332,-178},
          {-332,-216},{-322,-216}},color={255,0,255}));
  connect(TChiWatSupSet, capReq.TChiWatSupSet) annotation (Line(points={{-422,370},
          {-370,370},{-370,319},{-324,319}}, color={0,0,127}));
  connect(TChiWatRet, capReq.TChiWatRet) annotation (Line(points={{-422,300},{-374,
          300},{-374,314},{-324,314}}, color={0,0,127}));
  connect(VChiWat_flow, capReq.VChiWat_flow) annotation (Line(points={{-422,270},
          {-366,270},{-366,309},{-324,309}}, color={0,0,127}));
  connect(conf.yDesCap, cap.uDesCap) annotation (Line(points={{-338,-162},{-322,
          -162},{-322,-161},{-272,-161}}, color={0,0,127}));
  connect(conf.yMinCap, cap.uMinCap) annotation (Line(points={{-338,-166},{-312,
          -166},{-312,-164},{-272,-164}}, color={0,0,127}));
  connect(sta.yAvaUp, cap.uUp) annotation (Line(points={{-298,-203},{-286,-203},
          {-286,-170},{-272,-170}}, color={255,127,0}));
  connect(sta.yAvaDow, cap.uDown) annotation (Line(points={{-298,-206},{-282,-206},
          {-282,-173},{-272,-173}}, color={255,127,0}));
  connect(sta.yHig, cap.uHig) annotation (Line(points={{-298,-211},{-280,-211},
          {-280,-176},{-272,-176}},color={255,0,255}));
  connect(capReq.y, PLRs.uCapReq) annotation (Line(points={{-300,310},{-194,310},
          {-194,-176},{-184,-176}},
                                  color={0,0,127}));
  connect(cap.yDes, PLRs.uCapDes) annotation (Line(points={{-248,-162},{-234,
          -162},{-234,-178},{-184,-178}},
                                  color={0,0,127}));
  connect(cap.yUpDes, PLRs.uUpCapDes) annotation (Line(points={{-248,-166},{
          -236,-166},{-236,-180},{-184,-180}},
                                  color={0,0,127}));
  connect(cap.yDowDes, PLRs.uDowCapDes) annotation (Line(points={{-248,-170},{
          -238,-170},{-238,-182},{-184,-182}},
                                       color={0,0,127}));
  connect(cap.yMin, PLRs.uCapMin) annotation (Line(points={{-248,-174},{-240,
          -174},{-240,-185},{-184,-185}},
                                  color={0,0,127}));
  connect(cap.yUpMin, PLRs.uUpCapMin) annotation (Line(points={{-248,-178},{
          -242,-178},{-242,-187},{-184,-187}},
                                       color={0,0,127}));
  connect(uLif, PLRs.uLif) annotation (Line(points={{-422,50},{-202,50},{-202,-190},
          {-184,-190}},     color={0,0,127}));
  connect(uLifMax, PLRs.uLifMax) annotation (Line(points={{-422,20},{-212,20},{-212,
          -192},{-184,-192}},    color={0,0,127}));
  connect(uLifMin, PLRs.uLifMin) annotation (Line(points={{-422,-10},{-222,-10},
          {-222,-194},{-184,-194}},
                                 color={0,0,127}));
  connect(conf.yTyp, PLRs.uTyp) annotation (Line(points={{-338,-174},{-302,-174},
          {-302,-198},{-184,-198}},                    color={255,127,0}));
  connect(sta.yAvaUp, PLRs.uUp) annotation (Line(points={{-298,-203},{-242,-203},
          {-242,-204},{-184,-204}}, color={255,127,0}));
  connect(sta.yAvaDow, PLRs.uDown)
    annotation (Line(points={{-298,-206},{-184,-206}}, color={255,127,0}));
  connect(sta.yLow, cap.uLow) annotation (Line(points={{-298,-214},{-278,-214},
          {-278,-179},{-272,-179}},color={255,0,255}));
  connect(PLRs.yOpe, staUp.uOpe) annotation (Line(points={{-160,-182},{-136,-182},
          {-136,-100},{-102,-100}},
                             color={0,0,127}));
  connect(PLRs.yStaUp, staUp.uStaUp) annotation (Line(points={{-160,-191},{-134,
          -191},{-134,-102},{-102,-102}},
                                color={0,0,127}));
  connect(TChiWatSupSet, staUp.TChiWatSupSet) annotation (Line(points={{-422,370},
          {-162,370},{-162,-105},{-102,-105}},
                                       color={0,0,127}));
  connect(TChiWatSup, staUp.TChiWatSup) annotation (Line(points={{-422,340},{-382,
          340},{-382,270},{-164,270},{-164,-107},{-102,-107}},
                                                         color={0,0,127}));
  connect(dpChiWatPumSet, staUp.dpChiWatPumSet) annotation (Line(points={{-422,220},
          {-144,220},{-144,-110},{-102,-110}},
                                         color={0,0,127}));
  connect(dpChiWatPum, staUp.dpChiWatPum) annotation (Line(points={{-422,190},{-146,
          190},{-146,-112},{-102,-112}},
                                     color={0,0,127}));
  connect(PLRs.yOpeDow, staDow.uOpeDow) annotation (Line(points={{-160,-186},{-142,
          -186},{-142,-220},{-102,-220}},
                                color={0,0,127}));
  connect(staDow.uStaDow, PLRs.yStaDow) annotation (Line(points={{-102,-222},{-144,
          -222},{-144,-193},{-160,-193}},
                               color={0,0,127}));
  connect(dpChiWatPumSet, staDow.dpChiWatPumSet) annotation (Line(points={{-422,
          220},{-146,220},{-146,-225},{-102,-225}},
                                              color={0,0,127}));
  connect(dpChiWatPum, staDow.dpChiWatPum) annotation (Line(points={{-422,190},{
          -150,190},{-150,-227},{-102,-227}},
                                        color={0,0,127}));
  connect(TChiWatSupSet, staDow.TChiWatSupSet) annotation (Line(points={{-422,370},
          {-152,370},{-152,-230},{-102,-230}},
                                         color={0,0,127}));
  connect(TChiWatSup, staDow.TChiWatSup) annotation (Line(points={{-422,340},{-382,
          340},{-382,270},{-154,270},{-154,-232},{-102,-232}},
                                                       color={0,0,127}));
  connect(TWsePre, staDow.TWsePre) annotation (Line(points={{-422,120},{-156,120},
          {-156,-234},{-102,-234}},
                            color={0,0,127}));
  connect(uTowFanSpeMax, staDow.uTowFanSpeMax) annotation (Line(points={{-422,90},
          {-158,90},{-158,-236},{-102,-236}},
                                      color={0,0,127}));
  connect(staDow.uWseSta, uWseSta) annotation (Line(points={{-102,-241},{-340,-241},
          {-340,-260},{-422,-260}}, color={255,0,255}));
  connect(u, sta.u) annotation (Line(points={{-420,-60},{-328,-60},{-328,-204},{
          -322,-204}},  color={255,127,0}));
  connect(sta.yAvaCur, staUp.uAvaCur) annotation (Line(points={{-298,-217},{-242,
          -217},{-242,-210},{-122,-210},{-122,-119},{-102,-119}},
                                                           color={255,0,255}));
  connect(u, cap.u) annotation (Line(points={{-420,-60},{-328,-60},{-328,-167},{
          -272,-167}},  color={255,127,0}));
  connect(u, PLRs.u) annotation (Line(points={{-420,-60},{-232,-60},{-232,-202},
          {-184,-202}},                            color={255,127,0}));
  connect(u, staUp.u) annotation (Line(points={{-420,-60},{-112,-60},{-112,-116},
          {-102,-116}},                         color={255,127,0}));
  connect(u, staDow.u) annotation (Line(points={{-420,-60},{-328,-60},{-328,-239},
          {-102,-239}},     color={255,127,0}));
  connect(chaPro, capReq.chaPro) annotation (Line(points={{-420,-140},{-350,-140},
          {-350,302},{-324,302}},       color={255,0,255}));
  connect(sta.yAvaUp, cha.uAvaUp) annotation (Line(points={{-298,-203},{-292,-203},
          {-292,-148},{-40,-148},{-40,-164},{-22,-164}}, color={255,127,0}));
  connect(sta.yAvaDow, cha.uAvaDow) annotation (Line(points={{-298,-206},{-290,-206},
          {-290,-150},{-42,-150},{-42,-168},{-22,-168}}, color={255,127,0}));
  connect(staUp.y, cha.uUp) annotation (Line(points={{-78,-110},{-50,-110},{-50,
          -172},{-22,-172}}, color={255,0,255}));
  connect(staDow.y, cha.uDow) annotation (Line(points={{-78,-230},{-50,-230},{-50,
          -176},{-22,-176}}, color={255,0,255}));
  connect(uPla, cha.uPla) annotation (Line(points={{-420,-100},{-280,-100},{-280,
          -140},{-60,-140},{-60,-180},{-22,-180}},
                                             color={255,0,255}));
  connect(cha.ySta, ySta) annotation (Line(points={{2,-166},{20,-166},{20,-120},
          {140,-120}},
                     color={255,127,0}));
  connect(cha.yChaEdg, yChaEdg) annotation (Line(points={{2,-174},{114,-174},{
          114,-180},{140,-180}}, color={255,0,255}));
  connect(chiInd.yChi, yChiSet) annotation (Line(points={{62,-200},{80,-200},{
          80,-260},{140,-260}}, color={255,0,255}));
  connect(cha.ySta, chiInd.u) annotation (Line(points={{2,-166},{20,-166},{20,-200},
          {38,-200}},        color={255,127,0}));
  connect(TChiWatSupSet, iniSta.TChiWatSupSet) annotation (Line(points={{-422,370},
          {-100,370},{-100,113},{-82,113}}, color={0,0,127}));
  connect(uTunPar, iniSta.uTunPar) annotation (Line(points={{-420,150},{-104,150},
          {-104,116},{-82,116}}, color={0,0,127}));
  connect(iniSta.yIni, cha.uIni) annotation (Line(points={{-59,110},{-28,110},{-28,
          -160},{-22,-160}}, color={255,127,0}));
  connect(TOutWet, iniSta.TOutWet) annotation (Line(points={{-420,400},{-96,400},
          {-96,119},{-82,119}}, color={0,0,127}));
  connect(sta.yAvaUp, iniSta.uUp) annotation (Line(points={{-298,-203},{-294,-203},
          {-294,110},{-82,110}}, color={255,127,0}));
  annotation (defaultComponentName = "staSetCon",
        Icon(coordinateSystem(extent={{-100,-160},{100,160}}, initialScale=0.2),
        graphics={
        Rectangle(
        extent={{-100,-200},{100,180}},
        lineColor={0,0,127},
        fillColor={255,255,85},
        fillPattern=FillPattern.Solid),
        Text(
          extent={{-112,226},{108,188}},
          lineColor={0,0,255},
          textString="%name"),
        Text(
          extent={{-96,186},{-8,152}},
          lineColor={0,0,127},
          pattern=LinePattern.Dash,
          textString="TChiWatSupSet"),
        Text(
          extent={{-96,166},{-24,134}},
          lineColor={0,0,127},
          pattern=LinePattern.Dash,
          textString="TChiWatSup"),
        Text(
          extent={{-96,-42},{-50,-60}},
          lineColor={0,0,127},
          pattern=LinePattern.Dash,
          textString="TWsePre"),
        Text(
          extent={{-100,-26},{-32,-36}},
          lineColor={0,0,127},
          pattern=LinePattern.Dash,
          textString="TChiWatRet"),
        Text(
          extent={{-96,-56},{-32,-84}},
          lineColor={0,0,127},
          pattern=LinePattern.Dash,
          textString="TChiWat_flow"),
        Text(
          extent={{-96,18},{-12,-18}},
          lineColor={0,0,127},
          pattern=LinePattern.Dash,
          textString="uTowFanSpeMax"),
        Text(
          extent={{-94,50},{-4,10}},
          lineColor={0,0,127},
          pattern=LinePattern.Dash,
          textString="dpChiWatPumSet"),
        Text(
          extent={{-94,66},{-26,36}},
          lineColor={0,0,127},
          pattern=LinePattern.Dash,
          textString="dpChiWatPum"),
        Text(
          extent={{-96,88},{-56,72}},
          lineColor={0,0,127},
          pattern=LinePattern.Dash,
          textString="uLifMin"),
        Text(
          extent={{-96,112},{-52,90}},
          lineColor={0,0,127},
          pattern=LinePattern.Dash,
          textString="uLifMax"),
        Text(
          extent={{-100,126},{-70,114}},
          lineColor={0,0,127},
          pattern=LinePattern.Dash,
          textString="uLif"),
        Text(
          extent={{-104,-86},{-82,-96}},
          lineColor={244,125,35},
          pattern=LinePattern.Dash,
          textString="u"),
        Text(
          extent={{-98,-116},{-70,-104}},
          lineColor={244,125,35},
          pattern=LinePattern.Dash,
          textString="uIni"),
        Text(
          extent={{-96,-120},{-50,-142}},
          lineColor={217,67,180},
          pattern=LinePattern.Dash,
          textString="uWseSta"),
        Text(
          extent={{-96,-142},{-60,-158}},
          lineColor={217,67,180},
          pattern=LinePattern.Dash,
          textString="chaPro"),
        Text(
          extent={{-98,-184},{-68,-196}},
          lineColor={217,67,180},
          pattern=LinePattern.Dash,
          textString="uPla"),
        Text(
          extent={{-96,-162},{-50,-180}},
          lineColor={217,67,180},
          pattern=LinePattern.Dash,
          textString="uChiAva"),
        Text(
          extent={{66,64},{98,80}},
          lineColor={244,125,35},
          pattern=LinePattern.Dash,
          textString="ySta"),
        Text(
          extent={{72,8},{112,-8}},
          lineColor={217,67,180},
          pattern=LinePattern.Dash,
          textString="y"),
        Text(
          extent={{62,-62},{96,-78}},
          lineColor={217,67,180},
          pattern=LinePattern.Dash,
          textString="yChi")}),  Diagram(
        coordinateSystem(preserveAspectRatio=false,
        extent={{-400,-300},{120,420}})),
Documentation(info="<html>
<p>
The sequence is a chiller stage status setpoint controller that outputs the 
chiller stage integer index <code>ySta</code>, chiller stage change trigger signal
<code>y</code> and a chiller status vector for the current stage <code>yChi</code>.
</p>
<p>
Implemented according to ASHRAE RP-1711 March 2020 Draft, section 5.2.4.1 - 17.
</p>
<p>
The controller contains the following subsequences:
</p>
<ul>
<li>
<a href=\"modelica://Buildings.Controls.OBC.ASHRAE.PrimarySystem.ChillerPlant.Staging.Subsequences.CapacityRequirement\">
Buildings.Controls.OBC.ASHRAE.PrimarySystem.ChillerPlant.Staging.Subsequences.CapacityRequirement</a> to calculate
the capacity requirement
</li>
<li>
<a href=\"modelica://Buildings.Controls.OBC.ASHRAE.PrimarySystem.ChillerPlant.Staging.Subsequences.Configurator\">
Buildings.Controls.OBC.ASHRAE.PrimarySystem.ChillerPlant.Staging.Subsequences.Configurator</a> to allow the user 
to provide the chiller plant configuration parameters such as chiller design and minimal capacities and types. It 
calculates the design and minimal stage capacities, stage type and stage availability
</li>
<li>
<a href=\"modelica://Buildings.Controls.OBC.ASHRAE.PrimarySystem.ChillerPlant.Staging.Subsequences.Status\">
Buildings.Controls.OBC.ASHRAE.PrimarySystem.ChillerPlant.Staging.Subsequences.Status</a> to calculate
for instance the next higher and lower available stages
</li>
<li>
<a href=\"modelica://Buildings.Controls.OBC.ASHRAE.PrimarySystem.ChillerPlant.Staging.Subsequences.Capacities\">
Buildings.Controls.OBC.ASHRAE.PrimarySystem.ChillerPlant.Staging.Subsequences.Capacities</a> to calculate
design and minimal stage capacities for current and next available higher and lower stage
</li>
<li>
<a href=\"modelica://Buildings.Controls.OBC.ASHRAE.PrimarySystem.ChillerPlant.Staging.Subsequences.PartLoadRatios\">
Buildings.Controls.OBC.ASHRAE.PrimarySystem.ChillerPlant.Staging.Subsequences.PartLoadRatios</a> to calculate
operating and staging part load ratios for current and next available higher and lower stage
</li>
<li>
<a href=\"modelica://Buildings.Controls.OBC.ASHRAE.PrimarySystem.ChillerPlant.Staging.Subsequences.Up\">
Buildings.Controls.OBC.ASHRAE.PrimarySystem.ChillerPlant.Staging.Subsequences.Up</a> to generate
a stage up signal
</li>
<li>
<a href=\"modelica://Buildings.Controls.OBC.ASHRAE.PrimarySystem.ChillerPlant.Staging.Subsequences.Down\">
Buildings.Controls.OBC.ASHRAE.PrimarySystem.ChillerPlant.Staging.Subsequences.Down</a> to generate
a stage down signal
</li>
<li>
<a href=\"modelica://Buildings.Controls.OBC.ASHRAE.PrimarySystem.ChillerPlant.Staging.Subsequences.Change\">
Buildings.Controls.OBC.ASHRAE.PrimarySystem.ChillerPlant.Staging.Subsequences.Change</a> to set the stage
based on the initial stage signal and stage up and down signals
</li>
<li>
<a href=\"modelica://Buildings.Controls.OBC.ASHRAE.PrimarySystem.ChillerPlant.Staging.Subsequences.ChillerIndices\">
Buildings.Controls.OBC.ASHRAE.PrimarySystem.ChillerPlant.Staging.Subsequences.ChillerIndices</a> to generate
the chiller index vector for a given stage
</li>
</ul>
</html>",
revisions="<html>
<ul>
<li>
April 14, 2020, by Milica Grahovac:<br/>
First implementation.
</li>
</ul>
</html>"));
end SetpointController;
