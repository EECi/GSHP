within Buildings.Fluid.HeatPumps.Examples;
model Cam_GSHP "Test model for heat pump based on Carnot efficiency"
  extends Modelica.Icons.Example;
  package Medium1 = Buildings.Media.Water "Medium model";
  package Medium2 =
      Buildings.Media.Antifreeze.Validation.BaseClasses.PropyleneGlycolWater (
            property_T = 300,
            X_a = 0.05)
            "Medium model";

  package Medium3 = Buildings.Media.Water "Medium model";
  parameter Real COP_nominal_GroEva = 4.42 "Nominal COP for cooling";
  parameter Real COP_nominal_ConGro = 3.33 "Nominal COP for heaing";
  parameter Modelica.Units.SI.Power P_nominal_ConGro=72.2E3
    "Nominal compressor power for condense and ground side (at y=1)";
  parameter Modelica.Units.SI.Power P_nominal_GroEva=64E3
    "Nominal compressor power for ground and evaporator side (at y=1)";
  parameter Modelica.Units.SI.TemperatureDifference dTEva_nominal=-6
    "Temperature difference evaporator outlet-inlet";
  parameter Modelica.Units.SI.TemperatureDifference dTCon_nominal=5
    "Temperature difference condenser outlet-inlet";
  parameter Modelica.Units.SI.TemperatureDifference dTGro_nominal_Eva=-3
    "Temperature difference ground outlet-inlet";//10 works was condenser -10 work as evaporator
  parameter Modelica.Units.SI.TemperatureDifference dTGro_nominal_Con=5
    "Temperature difference ground outlet-inlet";//10 works was condenser -10 work as evaporator

  parameter Modelica.Units.SI.MassFlowRate m2_flow_nominal_GCon=
   (P_nominal_GroEva * (COP_nominal_GroEva+1))/cp2_default/dTGro_nominal_Con;//(condenser part )
  parameter Modelica.Units.SI.MassFlowRate m2_flow_nominal_GEva=
  -P_nominal_ConGro*(COP_nominal_ConGro-1)/cp2_default/dTGro_nominal_Eva;
      //evaporator part

  parameter Modelica.Units.SI.MassFlowRate m3_flow_nominal=-P_nominal_GroEva*
      COP_nominal_GroEva/cp3_default/dTEva_nominal
    "Nominal mass flow rate at chilled water side";  //see as evaporator

  parameter Modelica.Units.SI.MassFlowRate m1_flow_nominal=P_nominal_ConGro*
      COP_nominal_ConGro/cp1_default/dTCon_nominal
    "Nominal mass flow rate at condenser water wide";

  Buildings.Fluid.HeatPumps.Carnot_y_Z heaPum(
    redeclare package Medium1 = Medium1,
    redeclare package Medium2 = Medium2,
    redeclare package Medium3 = Medium3,
    m2_flow_nominal_GCon= m2_flow_nominal_GCon,
    m2_flow_nominal_GEva= m2_flow_nominal_GEva,
    m1_flow_nominal= m1_flow_nominal,
    m3_flow_nominal= m3_flow_nominal,
    P_nominal_GroEva=P_nominal_GroEva,
    P_nominal_ConGro=P_nominal_ConGro,
    dTEva_nominal=dTEva_nominal,
    dTCon_nominal=dTCon_nominal,
    dTGro_nominal_Eva=dTGro_nominal_Eva,
    dTGro_nominal_Con=dTGro_nominal_Con,
    dp1_nominal=6000,
    dp2_nominal_geva=6000,
    dp2_nominal_gcon=6000,
    dp3_nominal=6000,
    energyDynamics=Modelica.Fluid.Types.Dynamics.FixedInitial,
    show_T=true,
    use_eta_Carnot_nominal=false,
    COP_nominal_GroEva=COP_nominal_GroEva,
    COP_nominal_ConGro=COP_nominal_ConGro,
    TCon_nominal=303.15,
    TGro_nominal=293.15,
    TEva_nominal=278.15) "Heat pump model"
    annotation (Placement(transformation(extent={{-2,-8},{18,12}})));
  Buildings.Fluid.Sources.MassFlowSource_T sou1(
    use_m_flow_in=true,                         nPorts=1,
    redeclare package Medium = Medium1,
    use_T_in=true,
    m_flow=m1_flow_nominal,
    T=298.15)
    annotation (Placement(transformation(extent={{-68,0},{-48,20}})));
  Buildings.Fluid.Sources.MassFlowSource_T sou2(
    use_m_flow_in=false,                        nPorts=1,
    redeclare package Medium = Medium2,
    use_T_in=true,
    m_flow=max(m2_flow_nominal_GCon,m2_flow_nominal_GEva),
    T=291.15)
    annotation (Placement(transformation(extent={{52,-20},{32,0}})));
  Buildings.Fluid.Sources.MassFlowSource_T sou3(
    use_m_flow_in=true,                         nPorts=1,
    redeclare package Medium = Medium3,
    use_T_in=true,
    m_flow=m3_flow_nominal,
    T=295.15)
    annotation (Placement(transformation(extent={{32,-76},{12,-56}})));
  Buildings.Fluid.Sources.Boundary_pT sin1(
    redeclare package Medium = Medium1, nPorts=1)
    annotation (Placement(transformation(extent={{10,-10},{-10,10}}, origin={78,38})));
  Buildings.Fluid.Sources.Boundary_pT sin2(
    redeclare package Medium = Medium2, nPorts=1)
    annotation (Placement(transformation(extent={{-10,-10},{10,10}}, origin={-78,-22})));
  Buildings.Fluid.Sources.Boundary_pT  sin3(
   redeclare package Medium =Medium3, nPorts=1)
    annotation (Placement(transformation(extent={{-10,-10},{10,10}}, origin={-74,-64})));

  final parameter Modelica.Units.SI.SpecificHeatCapacity cp1_default=
      Medium1.specificHeatCapacityCp(Medium1.setState_pTX(
      Medium1.p_default,
      Medium1.T_default,
      Medium1.X_default))
    "Specific heat capacity of medium 2 at default medium state";

  final parameter Modelica.Units.SI.SpecificHeatCapacity cp2_default=
      Medium2.specificHeatCapacityCp(Medium2.setState_pTX(
      Medium2.p_default,
      Medium2.T_default,
      Medium2.X_default))
    "Specific heat capacity of medium 2 at default medium state";

  final parameter Modelica.Units.SI.SpecificHeatCapacity cp3_default=
      Medium3.specificHeatCapacityCp(Medium3.setState_pTX(
      Medium3.p_default,
      Medium3.T_default,
      Medium3.X_default))
    "Specific heat capacity of medium 3 at default medium state";

  Modelica.Blocks.Sources.CombiTimeTable combiTimeTable(
    tableOnFile=true,
    table=[0,14,74.5,44.9,11.3,38.4,35.3; 1,13.9,74.5,44.9,9.9,38.2,35.4; 2,14,74.5,
        44.9,11.2,38.3,35.7; 3,13.9,74.5,45,9.8,38.4,35.4; 4,14,74.5,45,11.2,38.6,
        35.6; 5,14,74.5,45,9.8,38.4,35.7; 6,14.1,74.5,45,11.2,38.3,35.5; 7,13.9,
        74.5,45.1,9.7,38.4,35.1; 8,14,74.5,45.1,11.1,38.5,35.5; 9,13.9,74.5,45.1,
        10,38.2,35.4; 10,14,74.5,45.1,11.1,38.5,35.1; 11,13.9,74.5,45.1,10.4,38.7,
        35.3; 12,14.1,74.5,45.2,11.1,38.7,35.3; 13,13.9,74.5,45.2,10.2,38.6,35.6;
        14,14.1,74.5,45.2,11,38.3,35.4; 15,13.7,74.5,45.2,11.4,38.5,35.3; 16,14.1,
        74.5,45.3,10.8,38.5,35.2; 17,13.7,74.5,45.3,11.7,38.3,35.2; 18,14.1,74.5,
        45.3,10.9,38.5,36.2; 19,14,74.5,45.3,11,38.2,35.5; 20,14.1,74.5,45.3,11,
        38.1,35.9; 21,14.1,74.5,45.4,10.8,38.7,35.6; 22,14.1,74.5,45.4,10.7,38.3,
        35.8; 23,14.2,74.5,45.4,11.6,38.9,35.7; 24,14.1,74.5,45.4,10.5,37.8,35.9;
        25,14.1,74.5,45.4,11.5,38.3,36; 26,14.1,74.5,45.5,10.4,38.2,35.7; 27,14.2,
        74.5,45.5,11.5,37.8,35.7; 28,14.1,74.5,45.5,10.2,38.3,35.4; 29,14,74.5,40.8,
        11.3,38,35.9; 30,13.1,74.5,43.1,9.9,38.3,35.8; 31,12.5,74.5,43.2,11.1,38.2,
        35.8; 32,12.5,74.5,42.9,10.2,38.1,35.5; 33,12.4,74.5,44.2,11.1,38.6,35.3;
        34,12.2,74.5,43.8,10.9,38.4,35.4; 35,12.3,74.5,43.4,10.9,38.5,35.8; 36,12.2,
        74.5,43.6,11.5,38.3,35.3; 37,12.2,74.5,43.6,10.8,38.6,35.4; 38,12.1,74.5,
        44.1,11.6,38.7,35.6; 39,12.2,74.5,44.1,10.6,38.6,35.8; 40,12,74.5,44.7,11.6,
        38.1,35.4; 41,12.2,74.5,44.8,10.4,38.2,36.1; 42,12,74.5,45.1,11.4,37.9,35.8;
        43,12.5,74.5,44.4,10.3,38.6,35.6; 44,12.1,74.5,44,11.4,38.4,35.5; 45,12.3,
        74.5,45.3,10.1,38.2,35; 46,12.3,74.5,45.3,11.3,38.7,35.4; 47,12.4,74.5,45.1,
        10,38.5,35.1; 48,12.5,74.5,44.9,11.3,38.3,35.2; 49,12.5,74.5,44.7,9.9,38.4,
        35.7; 50,12.6,74.5,44.7,11.2,38.7,36; 51,12.5,74.5,44.4,10,38.2,35.9; 52,
        12.6,74.5,43.9,11.2,38.2,35.7; 53,12.7,74.5,43.8,9.8,38.3,35.4; 54,12.6,
        74.5,43.7,11.2,38.2,35.6; 55,12.8,74.5,43.6,9.7,38.3,35.8; 56,12.9,74.5,
        43.8,11.1,38.1,35.6; 57,12.7,74.5,44,10.3,38,35.3; 58,13,74.5,44.4,11,38.6,
        35.6; 59,12.7,74.5,44.7,10.7,38.7,35.6; 60,13,74.5,45.4,10.9,38.3,35.4;
        61,13,74.5,44.7,11.2,38.4,35.8; 62,13.1,74.5,43.8,10.9,38.5,35; 63,12.9,
        74.5,44.1,11.7,38.1,35.7; 64,13.3,74.5,44.6,10.8,38.4,35.7; 65,12.9,74.5,
        45.1,11.7,38.6,35.6; 66,13.2,74.5,45.5,10.7,37.9,35.5; 67,13.1,74.5,45.3,
        11.6,38.4,34.9; 68,13.4,74.5,44.1,10.5,38.4,36.4; 69,13.2,74.5,44.1,11.5,
        38.4,35.7; 70,13.3,74.5,45.3,10.4,38.5,35.5; 71,13.5,74.5,44,11.5,37.9,35.7;
        72,13.4,74.5,45.4,10.4,38.5,35.7; 73,13.3,74.5,43.8,11.5,38.3,35.7; 74,13.3,
        74.5,44.4,10.2,37.8,35.4; 75,13.3,74.5,45.5,11.4,38.6,35.9; 76,13.5,74.5,
        43.8,10.1,38.2,35.5; 77,13.4,74.5,44.4,11.3,38.3,35.4; 78,13.3,74.5,45.4,
        9.8,38.3,35.3; 79,13.4,74.5,45.5,11.1,38.3,35.4; 80,13.2,74.5,44.3,10.2,
        38.2,35.5; 81,13.3,74.5,43.7,11.1,38.3,35.8; 82,13.1,74.5,43.8,10.3,38.3,
        35.8; 83,13.4,74.5,44.1,10.9,38.3,35.4; 84,13.1,74.5,44.8,11.7,38.5,36;
        85,13.4,74.5,46.1,10.6,37.7,35.5; 86,13.5,74.5,46,11.6,38.8,35.2; 87,13.5,
        74.5,46,10.4,38.8,35.7; 88,13.6,74.5,46,11.5,38.5,35.9; 89,13.7,74.5,46,
        10.3,38.2,35.4; 90,13.7,74.5,46,11.6,38.3,35.1; 91,13.8,74.5,46,10.6,38.6,
        35.6; 92,13.9,74.5,46,11.7,38.4,35.4; 93,13.8,74.5,46,10.5,38.2,35.8; 94,
        13.9,74.5,46,11.5,38.6,35.3; 95,13.8,74.5,46.1,10.3,38.3,35.9; 96,13.9,74.5,
        46,11.4,38.2,36.1],
    tableName="table",
    fileName=ModelicaServices.ExternalReferences.loadResource("modelica://Buildings/GSHP_input_data.txt"),
    extrapolation=Modelica.Blocks.Types.Extrapolation.Periodic,
    timeScale=900)
    annotation (Placement(transformation(extent={{-212,-86},{-192,-66}})));

  Modelica.Fluid.Sensors.TemperatureTwoPort Ground_Entering_T(redeclare package
      Medium = Medium2)
    annotation (Placement(transformation(extent={{-26,-30},{-42,-6}})));
  Modelica.Fluid.Sensors.TemperatureTwoPort GSHP_to_Hot_Buffer_T(redeclare
      package Medium = Medium1)
    annotation (Placement(transformation(extent={{22,52},{42,74}})));
  Modelica.Fluid.Sensors.TemperatureTwoPort GSHP_to_Cool_Buffer_T(redeclare
      package Medium = Medium3)
    annotation (Placement(transformation(extent={{-22,-68},{-38,-44}})));
  Modelica.Blocks.Logical.Hysteresis hys(uLow=273.15 + 44, uHigh=273.15 + 46)
    "Hysteresis controller"
    annotation (Placement(transformation(extent={{-212,72},{-192,92}})));
  Modelica.Blocks.Math.BooleanToReal booToRea(realTrue=0, realFalse=1)
    "Conversion to real control signal"
    annotation (Placement(transformation(extent={{-172,72},{-152,92}})));
  Modelica.Blocks.Logical.Hysteresis hys1(uLow=273.15 + 9, uHigh=273.15 + 12)
    "Hysteresis controller"
    annotation (Placement(transformation(extent={{-208,30},{-188,50}})));
  Modelica.Blocks.Math.BooleanToReal booToRea1(realTrue=1, realFalse=0)
    "Conversion to real control signal"
    annotation (Placement(transformation(extent={{-172,30},{-152,50}})));
equation
  connect(sou1.ports[1], heaPum.port_a1) annotation (Line(
      points={{-48,10},{-22,10},{-22,5.33333},{-0.57143,5.33333}},
      color={0,127,255},
      smooth=Smooth.None));
  connect(sou2.ports[1], heaPum.port_a2) annotation (Line(
      points={{32,-10},{28,-10},{28,0.33333},{13.7143,0.33333}},
      color={0,127,255},
      smooth=Smooth.None));
  connect(sou3.ports[1], heaPum.port_a3)
    annotation (Line(points={{12,-66},{13.7143,-66},{13.7143,-4.83333}},
                                                          color={0,127,255}));
  connect(combiTimeTable.y[1], sou2.T_in) annotation (Line(points={{-191,-76},{
          -160,-76},{-160,-2},{-116,-2},{-116,18},{-110,18},{-110,28},{60,28},{
          60,-6},{54,-6}},
                color={0,0,127}));
  connect(combiTimeTable.y[4], sou3.T_in) annotation (Line(points={{-191,-76},{-78,
          -76},{-78,-84},{14,-84},{14,-86},{40,-86},{40,-62},{34,-62}}, color={0,
          0,127}));
  connect(combiTimeTable.y[3], sou1.T_in) annotation (Line(points={{-191,-76},{-160,
          -76},{-160,-2},{-116,-2},{-116,18},{-76,18},{-76,14},{-70,14}},
                                                     color={0,0,127}));
  connect(heaPum.port_b2, Ground_Entering_T.port_a) annotation (Line(points={{-0.428571,
          0.666667},{-8,0.666667},{-8,-18},{-26,-18}}, color={0,127,255}));
  connect(Ground_Entering_T.port_b, sin2.ports[1]) annotation (Line(points={{-42,-18},
          {-60,-18},{-60,-22},{-68,-22}},      color={0,127,255}));
  connect(heaPum.port_b3, GSHP_to_Cool_Buffer_T.port_a) annotation (Line(points={{
          -0.285714,-5.16667},{-0.285714,-4},{-12,-4},{-12,-56},{-22,-56}},
        color={0,127,255}));
  connect(GSHP_to_Cool_Buffer_T.port_b, sin3.ports[1]) annotation (Line(points={{-38,-56},
          {-46,-56},{-46,-64},{-64,-64}},           color={0,127,255}));
  connect(heaPum.port_b1, GSHP_to_Hot_Buffer_T.port_a) annotation (Line(points={{13.7143,
          5.33333},{13.7143,4},{22,4},{22,46},{8,46},{8,63},{22,63}},
        color={0,127,255}));
  connect(GSHP_to_Hot_Buffer_T.port_b, sin1.ports[1]) annotation (Line(points={{
          42,63},{42,60},{62,60},{62,38},{68,38}}, color={0,127,255}));
  connect(combiTimeTable.y[6], sou3.m_flow_in) annotation (Line(points={{-191,-76},
          {-78,-76},{-78,-84},{14,-84},{14,-86},{36,-86},{36,-84},{40,-84},{40,-58},
          {34,-58}},
        color={0,0,127}));
  connect(combiTimeTable.y[5], sou1.m_flow_in) annotation (Line(points={{-191,-76},
          {-160,-76},{-160,-2},{-116,-2},{-116,18},{-70,18}},
                                        color={0,0,127}));
  connect(booToRea.y, heaPum.y_ConGro) annotation (Line(points={{-151,82},{-10,82},
          {-10,2.75},{-0.571429,2.75}}, color={0,0,127}));
  connect(hys.y, booToRea.u)
    annotation (Line(points={{-191,82},{-174,82}}, color={255,0,255}));
  connect(combiTimeTable.y[3], hys.u) annotation (Line(points={{-191,-76},{-186,
          -76},{-186,-2},{-220,-2},{-220,82},{-214,82}}, color={0,0,127}));
  connect(booToRea1.y, heaPum.y_GroEva) annotation (Line(points={{-151,40},{-36,
          40},{-36,2},{-12,2},{-12,-1.83333},{-0.714286,-1.83333}}, color={0,0,127}));
  connect(combiTimeTable.y[4], hys1.u) annotation (Line(points={{-191,-76},{-186,
          -76},{-186,-2},{-220,-2},{-220,40},{-210,40}}, color={0,0,127}));
  connect(hys1.y, booToRea1.u)
    annotation (Line(points={{-187,40},{-174,40}}, color={255,0,255}));
  annotation (experiment(
      StopTime=86400,
      Interval=900,
      Tolerance=1e-06,
      __Dymola_Algorithm="Dassl"),
__Dymola_Commands(file="modelica://Buildings/Resources/Scripts/Dymola/Fluid/HeatPumps/Examples/Carnot_y.mos"
        "Simulate and plot"),
    Documentation(revisions="<html>
<ul>
<li>
May 15, 2019, by Jianjun Hu:<br/>
Replaced fluid source. This is for 
<a href=\"https://github.com/ibpsa/modelica-ibpsa/issues/1072\"> #1072</a>.
</li>
<li>
November 25, 2015 by Michael Wetter:<br/>
Changed sign of <code>dTEva_nominal</code> to be consistent.
</li>
<li>
December 22, 2014 by Michael Wetter:<br/>
Removed <code>Modelica.Fluid.System</code>
to address issue
<a href=\"https://github.com/lbl-srg/modelica-buildings/issues/311\">#311</a>.
</li>
<li>
March 26, 2013 by Michael Wetter:<br/>
Removed assignment of parameter that had attribute <code>fixed=false</code>.
</li>
<li>
March 3, 2009 by Michael Wetter:<br/>
First implementation.
</li>
</ul>
</html>", info="<html>
<p>
Example that simulates a heat pump whose efficiency is scaled based on the
Carnot cycle.
The control signal of the heat pump is the compressor speed.
</p>
</html>"),
    Diagram(coordinateSystem(extent={{-260,-120},{160,100}}), graphics={
        Text(
          extent={{28,-14},{52,-28}},
          textColor={28,108,200},
          textString="Water temperature from ground side
"),     Text(
          extent={{56,-80},{-20,-76}},
          textColor={28,108,200},
          textString="Water temperature from cool tank"),
        Text(
          extent={{-78,-38},{-70,-40}},
          textColor={28,108,200},
          textString="Ground side sink
"),     Text(
          extent={{-66,-68},{-54,-94}},
          textColor={28,108,200},
          textString="Building cooling zone sink
"),     Text(
          extent={{-76,10},{-150,6}},
          textColor={28,108,200},
          textString="Water temperature from hot tank"),
        Text(
          extent={{-172,100},{-246,96}},
          textColor={28,108,200},
          textString="Water temperature from hot tank"),
        Text(
          extent={{-166,22},{-242,26}},
          textColor={28,108,200},
          textString="Water temperature from cool tank"),
        Text(
          extent={{112,20},{36,24}},
          textColor={28,108,200},
          textString="Building heating zone sink
")}),
    Icon(coordinateSystem(extent={{-260,-120},{160,100}})));
end Cam_GSHP;
