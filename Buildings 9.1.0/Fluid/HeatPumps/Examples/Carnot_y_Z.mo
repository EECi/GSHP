within Buildings.Fluid.HeatPumps.Examples;
model Carnot_y_Z
  "Test model for heat pump based on Carnot efficiency"
  extends Modelica.Icons.Example;
  package Medium1 = Buildings.Media.Water "Medium model";
  package Medium2 = Buildings.Media.Water "Medium model";
  package Medium3 = Buildings.Media.Water "Medium model";
  parameter Real COP_nominal_GroEva = 5 "Nominal COP for cooling";
  parameter Real COP_nominal_ConGro = 6 "Nominal COP for heaing";
  parameter Modelica.Units.SI.Power P_nominal_ConGro=10E3
    "Nominal compressor power for condense and ground side (at y=1)";
  parameter Modelica.Units.SI.Power P_nominal_GroEva=10E3
    "Nominal compressor power for ground and evaporator side (at y=1)";
  parameter Modelica.Units.SI.TemperatureDifference dTEva_nominal=-10
    "Temperature difference evaporator outlet-inlet";
  parameter Modelica.Units.SI.TemperatureDifference dTCon_nominal=10
    "Temperature difference condenser outlet-inlet";
  parameter Modelica.Units.SI.TemperatureDifference dTGro_nominal_Eva=-10
    "Temperature difference ground outlet-inlet";//10 works was condenser -10 work as evaporator
  parameter Modelica.Units.SI.TemperatureDifference dTGro_nominal_Con=10
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
  Buildings.Fluid.Sources.MassFlowSource_T sou1(nPorts=1,
    redeclare package Medium = Medium1,
    use_T_in=true,
    m_flow=m1_flow_nominal,
    T=298.15)
    annotation (Placement(transformation(extent={{-62,-2},{-42,18}})));
  Buildings.Fluid.Sources.MassFlowSource_T sou2(nPorts=1,
    redeclare package Medium = Medium2,
    use_T_in=true,
    m_flow=max(m2_flow_nominal_GCon,m2_flow_nominal_GEva),
    T=291.15)
    annotation (Placement(transformation(extent={{58,-22},{38,-2}})));
  Buildings.Fluid.Sources.MassFlowSource_T sou3(nPorts=1,
    redeclare package Medium = Medium3,
    use_T_in=true,
    m_flow=m3_flow_nominal,
    T=295.15)
    annotation (Placement(transformation(extent={{56,-76},{36,-56}})));
  Buildings.Fluid.Sources.Boundary_pT sin1(
    nPorts=1,
    redeclare package Medium = Medium1)
    annotation (Placement(transformation(extent={{10,-10},{-10,10}}, origin={48,32})));
  Buildings.Fluid.Sources.Boundary_pT sin2(
    nPorts=1,
    redeclare package Medium = Medium2)
    annotation (Placement(transformation(extent={{-10,-10},{10,10}}, origin={-64,-28})));
  Buildings.Fluid.Sources.Boundary_pT  sin3(
   nPorts=1,
   redeclare package Medium =Medium3)
    annotation (Placement(transformation(extent={{-10,-10},{10,10}}, origin={-64,-64})));

  Modelica.Blocks.Sources.Ramp TCon_in(
    height=10,
    duration=60,
    offset=273.15 + 30,
    startTime=60) "Condenser inlet temperature"
    annotation (Placement(transformation(extent={{-98,-2},{-78,18}})));
  Modelica.Blocks.Sources.Ramp TGro_in(
    height=10,
    duration=60,
    startTime=600,
    offset=273.15 + 20) "Ground inlet temperature"
    annotation (Placement(transformation(extent={{72,-38},{92,-18}})));
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

 Modelica.Blocks.Sources.Ramp uCom_GroEva(
    height=-1,
    duration=60,
    offset=1,
    startTime=1800) "Compressor control signal"
    annotation (Placement(transformation(extent={{-98,38},{-78,58}})));

  Modelica.Blocks.Sources.Ramp uCom_ConGro(
    height=-1,
    duration=60,
    offset=1,
    startTime=1800) "Compressor control signal"
    annotation (Placement(transformation(extent={{-98,70},{-78,90}})));

  Modelica.Blocks.Sources.Ramp TEva_in(
    height=10,
    duration=60,
    startTime=900,
    offset=273.15 + 15) "Evaporator inlet temperature"
    annotation (Placement(transformation(extent={{74,-90},{94,-70}})));

equation
  connect(sou1.ports[1], heaPum.port_a1) annotation (Line(
      points={{-42,8},{-22,8},{-22,5.33333},{-0.57143,5.33333}},
      color={0,127,255},
      smooth=Smooth.None));
  connect(sou2.ports[1], heaPum.port_a2) annotation (Line(
      points={{38,-12},{28,-12},{28,0.33333},{13.7143,0.33333}},
      color={0,127,255},
      smooth=Smooth.None));
  connect(heaPum.port_b1, sin1.ports[1]) annotation (Line(
      points={{13.7143,5.33333},{28,5.33333},{28,32},{38,32}},
      color={0,127,255},
      smooth=Smooth.None));
  connect(sin2.ports[1], heaPum.port_b2) annotation (Line(
      points={{-54,-28},{-8,-28},{-8,0.66667},{-0.42857,0.66667}},
      color={0,127,255},
      smooth=Smooth.None));
  connect(TCon_in.y, sou1.T_in) annotation (Line(
      points={{-77,8},{-70,8},{-70,12},{-64,12}},
      color={0,0,127},
      smooth=Smooth.None));
  connect(TGro_in.y, sou2.T_in) annotation (Line(
      points={{93,-28},{96,-28},{96,-8},{60,-8}},
      color={0,0,127},
      smooth=Smooth.None));
  connect(uCom_GroEva.y, heaPum.y_GroEva) annotation (Line(
      points={{-77,48},{-12,48},{-12,-1.83333},{-0.71429,-1.83333}},
      color={0,0,127},
      smooth=Smooth.None));
  connect(TEva_in.y, sou3.T_in) annotation (Line(
      points={{95,-80},{100,-80},{100,-62},{58,-62}},
      color={0,0,127},
      smooth=Smooth.None));
  connect(sou3.ports[1], heaPum.port_a3)
    annotation (Line(points={{36,-66},{13.7143,-66},{13.7143,-4.83333}},
                                                          color={0,127,255}));
  connect(heaPum.port_b3, sin3.ports[1]) annotation (Line(points={{-0.28571,
          -5.16667},{-0.28571,-6},{-6,-6},{-6,-64},{-54,-64}},
                                         color={0,127,255}));
  connect(uCom_ConGro.y, heaPum.y_ConGro) annotation (Line(points={{-77,80},{
          -0.57143,80},{-0.57143,2.75}},
                            color={0,0,127}));
  annotation (experiment(Tolerance=1e-6, StopTime=3600),
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
</html>"));
end Carnot_y_Z;
