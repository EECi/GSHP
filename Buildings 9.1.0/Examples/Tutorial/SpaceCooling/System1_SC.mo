within Buildings.Examples.Tutorial.SpaceCooling;
model System1_SC
  "First part of the system model, consisting of the room with heat transfer"
  extends Modelica.Icons.Example;
  replaceable package MediumA = Buildings.Media.Air "Medium for air";
  replaceable package MediumW = Buildings.Media.Water "Medium for water";
  Buildings.Fluid.MixingVolumes.MixingVolume vol(
    redeclare package Medium = MediumA,
    m_flow_nominal=mA_flow_nominal,
    V=V,
    energyDynamics=Modelica.Fluid.Types.Dynamics.FixedInitial,
    mSenFac=1,
    nPorts=2)
    annotation (Placement(transformation(extent={{82,26},{102,46}})));
  Modelica.Thermal.HeatTransfer.Components.ThermalConductor theCon(G=10000/30)
    "Thermal conductance with the ambient"
    annotation (Placement(transformation(extent={{-10,22},{10,42}})));
  parameter Modelica.Units.SI.Volume V=6*10*3 "Room volume";
  parameter Modelica.Units.SI.MassFlowRate mA_flow_nominal=
    1.3*QRooC_flow_nominal/1006/(TASup_nominal-TRooSet)
     "Nominal air mass flow rate, increased by factor 1.3 to allow for recovery after temperature setback";
  parameter Modelica.Units.SI.HeatFlowRate QRooInt_flow=1000
    "Internal heat gains of the room";
    //////////////////////////////////////////////////////////
  // Heat recovery effectiveness
  parameter Real eps = 0.8 "Heat recovery effectiveness";

  /////////////////////////////////////////////////////////
  // Air temperatures at design conditions
  parameter Modelica.Units.SI.Temperature TASup_nominal = 273.15+18
    "Nominal air temperature supplied to room";
  parameter Modelica.Units.SI.Temperature TRooSet = 273.15+24
    "Nominal room air temperature";
  parameter Modelica.Units.SI.Temperature TOut_nominal = 273.15+30
    "Design outlet air temperature";
  parameter Modelica.Units.SI.Temperature THeaRecLvg=
    TOut_nominal - eps*(TOut_nominal-TRooSet)
    "Air temperature leaving the heat recovery";

  /////////////////////////////////////////////////////////
  // Cooling loads and air mass flow rates

  parameter Modelica.Units.SI.HeatFlowRate QRooC_flow_nominal=
    -QRooInt_flow-10E3/30*(TOut_nominal-TRooSet)
    "Nominal cooling load of the room";

  parameter Modelica.Units.SI.TemperatureDifference dTFan = 2
    "Estimated temperature raise across fan that needs to be made up by the cooling coil";
  parameter Modelica.Units.SI.HeatFlowRate QCoiC_flow_nominal=4*
    (QRooC_flow_nominal + mA_flow_nominal*(TASup_nominal-THeaRecLvg-dTFan)*1006)
    "Cooling load of coil, taking into account economizer, and increased due to latent heat removal";

  /////////////////////////////////////////////////////////
  // Water temperatures and mass flow rates
  parameter Modelica.Units.SI.Temperature TWSup_nominal = 273.15+16
    "Water supply temperature";
  parameter Modelica.Units.SI.Temperature TWRet_nominal = 273.15+12
    "Water return temperature";
  parameter Modelica.Units.SI.MassFlowRate mW_flow_nominal=
    QCoiC_flow_nominal/(TWRet_nominal-TWSup_nominal)/4200
    "Nominal water mass flow rate";
  Modelica.Thermal.HeatTransfer.Sources.FixedHeatFlow preHea(Q_flow=
        QRooInt_flow) "Prescribed heat flow"
    annotation (Placement(transformation(extent={{8,82},{28,102}})));
  Modelica.Thermal.HeatTransfer.Components.HeatCapacitor heaCap(C=2*V*1.2*1006)
    "Heat capacity for furniture and walls"
    annotation (Placement(transformation(extent={{52,68},{72,88}})));
  BoundaryConditions.WeatherData.ReaderTMY3 weaDat(filNam=
        Modelica.Utilities.Files.loadResource("modelica://Buildings/Resources/weatherdata/USA_IL_Chicago-OHare.Intl.AP.725300_TMY3.mos"),
    pAtmSou=Buildings.BoundaryConditions.Types.DataSource.Parameter,
    TDryBulSou=Buildings.BoundaryConditions.Types.DataSource.File)
    "Weather data reader"
    annotation (Placement(transformation(extent={{-104,84},{-84,104}})));
  BoundaryConditions.WeatherData.Bus weaBus
    annotation (Placement(transformation(extent={{-44,84},{-24,104}})));
  Modelica.Thermal.HeatTransfer.Sources.PrescribedTemperature TOut
    "Outside temperature"
    annotation (Placement(transformation(extent={{-52,20},{-32,40}})));
  Fluid.Movers.FlowControlled_m_flow fan(
    redeclare package Medium = MediumA,
    energyDynamics=Modelica.Fluid.Types.Dynamics.FixedInitial,
    m_flow_nominal=mA_flow_nominal)
    annotation (Placement(transformation(extent={{76,-74},{96,-54}})));
  Fluid.HeatExchangers.ConstantEffectiveness hex(
    redeclare package Medium1 = MediumA,
    redeclare package Medium2 = MediumA,
    m1_flow_nominal=mA_flow_nominal,
    m2_flow_nominal=mA_flow_nominal,
    dp1_nominal=200,
    dp2_nominal=200,
    eps=eps) annotation (Placement(transformation(extent={{-58,-26},{-38,-6}})));
  Fluid.HeatExchangers.WetCoilEffectivenessNTU cooCoi(
    redeclare package Medium1 = MediumW,
    redeclare package Medium2 = MediumA,
    m1_flow_nominal=mW_flow_nominal,
    m2_flow_nominal=mA_flow_nominal,
    show_T=true,
    dp1_nominal=6000,
    dp2_nominal=200,
    UA_nominal=-QCoiC_flow_nominal/
        Buildings.Fluid.HeatExchangers.BaseClasses.lmtd(
        T_a1=THeaRecLvg,
        T_b1=TASup_nominal,
        T_a2=TWSup_nominal,
        T_b2=TWRet_nominal),
    energyDynamics=Modelica.Fluid.Types.Dynamics.FixedInitial) "Cooling coil"
    annotation (Placement(transformation(extent={{22,-24},{8,-38}})));
  Controls.OBC.CDL.Continuous.Sources.Constant mAir_flow(k=mA_flow_nominal)
    "Fan air flow rate"
    annotation (Placement(transformation(extent={{76,-24},{96,-4}})));
  Fluid.Sources.Boundary_pT sinWat(redeclare package Medium = MediumW, nPorts=1)
    "Sink for water circuit"
    annotation (Placement(transformation(extent={{-28,-56},{-8,-36}})));
  Fluid.Sensors.TemperatureTwoPort senTemHXOut(redeclare package Medium =
        MediumA, m_flow_nominal=mA_flow_nominal)
    "Temperature sensor for heat recovery outlet on supply side"
    annotation (Placement(transformation(extent={{-28,-20},{-8,0}})));
  Fluid.Sensors.TemperatureTwoPort senTemSupAir(redeclare package Medium =
        MediumA, m_flow_nominal=mA_flow_nominal)
    "Temperature sensor for supply air"
    annotation (Placement(transformation(extent={{40,-20},{60,0}})));
  Fluid.Sources.Outside out(redeclare package Medium = MediumA, nPorts=2)
    annotation (Placement(transformation(extent={{-102,-24},{-82,-4}})));
  Fluid.Sources.MassFlowSource_T souWat(
    redeclare package Medium = MediumW,
    use_m_flow_in=true,
    T=TWSup_nominal,
    nPorts=1) "Source for water flow rate"
    annotation (Placement(transformation(extent={{-14,-96},{6,-76}})));
  Controls.OBC.CDL.Conversions.BooleanToReal mWat_flow(realTrue=0, realFalse=
        mW_flow_nominal) "Conversion from boolean to real for water flow rate"
    annotation (Placement(transformation(extent={{-90,-80},{-70,-60}})));
  Controls.OBC.CDL.Logical.OnOffController con(bandwidth=1)
    "Controller for coil water flow rate"
    annotation (Placement(transformation(extent={{-132,-80},{-112,-60}})));
  Controls.OBC.CDL.Continuous.Sources.Constant TRooSetPoi(k=TRooSet)
    "Room temperature set point"
    annotation (Placement(transformation(extent={{-174,-78},{-154,-58}})));
  Modelica.Thermal.HeatTransfer.Sensors.TemperatureSensor senTemRoo
    "Room temperature sensor"
    annotation (Placement(transformation(extent={{98,72},{118,92}})));
equation
  connect(theCon.port_b, vol.heatPort) annotation (Line(
      points={{10,32},{70,32},{70,36},{82,36}},
      color={191,0,0},
      smooth=Smooth.None));
  connect(preHea.port, vol.heatPort) annotation (Line(
      points={{28,92},{42,92},{42,44},{72,44},{72,36},{82,36}},
      color={191,0,0},
      smooth=Smooth.None));
  connect(heaCap.port, vol.heatPort) annotation (Line(points={{62,68},{62,44},{
          72,44},{72,36},{82,36}},
                                color={191,0,0}));
  connect(weaDat.weaBus,weaBus)  annotation (Line(
      points={{-84,94},{-34,94}},
      color={255,204,51},
      thickness=0.5,
      smooth=Smooth.None), Text(
      textString="%second",
      index=1,
      extent={{6,3},{6,3}}));
  connect(weaBus.TDryBul, TOut.T) annotation (Line(
      points={{-34,94},{-34,44},{-60,44},{-60,30},{-54,30}},
      color={255,204,51},
      thickness=0.5), Text(
      string="%first",
      index=-1,
      extent={{-3,6},{-3,6}},
      horizontalAlignment=TextAlignment.Right));
  connect(TOut.port, theCon.port_a) annotation (Line(points={{-32,30},{-30,30},
          {-30,32},{-10,32}},
                           color={191,0,0}));
  connect(weaDat.weaBus, out.weaBus) annotation (Line(
      points={{-84,94},{-96,94},{-96,12},{-122,12},{-122,-13.8},{-102,-13.8}},
      color={255,204,51},
      thickness=0.5));
  connect(out.ports[1], hex.port_a1) annotation (Line(points={{-82,-15},{-64,-15},
          {-64,-10},{-58,-10}}, color={0,127,255}));
  connect(out.ports[2], hex.port_b2) annotation (Line(points={{-82,-13},{-64,-13},
          {-64,-22},{-58,-22}}, color={0,127,255}));
  connect(hex.port_b1, senTemHXOut.port_a)
    annotation (Line(points={{-38,-10},{-28,-10}}, color={0,127,255}));
  connect(senTemSupAir.port_b, fan.port_a) annotation (Line(points={{60,-10},{70,
          -10},{70,-64},{76,-64}}, color={0,127,255}));
  connect(mAir_flow.y, fan.m_flow_in) annotation (Line(points={{98,-14},{104,-14},
          {104,-46},{86,-46},{86,-52}}, color={0,0,127}));
  connect(hex.port_a2, vol.ports[1]) annotation (Line(points={{-38,-22},{-38,16},
          {-28,16},{-28,18},{91,18},{91,26}},        color={0,127,255}));
  connect(fan.port_b, vol.ports[2]) annotation (Line(points={{96,-64},{96,20},{
          93,20},{93,26}},           color={0,127,255}));
  connect(senTemHXOut.port_b, cooCoi.port_a2) annotation (Line(points={{-8,-10},
          {2,-10},{2,-26.8},{8,-26.8}}, color={0,127,255}));
  connect(sinWat.ports[1], cooCoi.port_b1) annotation (Line(points={{-8,-46},{2,
          -46},{2,-35.2},{8,-35.2}}, color={0,127,255}));
  connect(cooCoi.port_b2, senTemSupAir.port_a) annotation (Line(points={{22,
          -26.8},{32,-26.8},{32,-10},{40,-10}}, color={0,127,255}));
  connect(souWat.ports[1], cooCoi.port_a1) annotation (Line(points={{6,-86},{32,
          -86},{32,-42},{22,-42},{22,-35.2}}, color={0,127,255}));
  connect(vol.heatPort, senTemRoo.port) annotation (Line(points={{82,36},{72,36},
          {72,64},{92,64},{92,82},{98,82}}, color={191,0,0}));
  connect(TRooSetPoi.y, con.reference) annotation (Line(points={{-152,-68},{
          -142,-68},{-142,-64},{-134,-64}}, color={0,0,127}));
  connect(senTemRoo.T, con.u) annotation (Line(points={{119,82},{124,82},{124,
          -100},{-140,-100},{-140,-76},{-134,-76}}, color={0,0,127}));
  connect(con.y, mWat_flow.u)
    annotation (Line(points={{-110,-70},{-92,-70}}, color={255,0,255}));
  connect(mWat_flow.y, souWat.m_flow_in) annotation (Line(points={{-68,-70},{
          -22,-70},{-22,-78},{-16,-78}}, color={0,0,127}));
  annotation (Documentation(info="<html>
<p>
This part of the system model implements the room with a heat gain.
The room is simplified as a volume of air, a prescribed heat source for
the internal convective heat gain, and a heat conductor for steady-state
heat conduction to the outside.
To increase the heat capacity of the room, such as due to heat stored
in furniture and in building constructions, the heat capacity
of the room air was increased by a factor of three.
The convective heat transfer coefficient is lumped into the heat conductor
model.
</p>
<h4>Implementation</h4>
<p>
This section describes step by step how we implemented the model.
</p>
<ol>
<li>
<p>
First, to define the medium properties, we added the declaration
</p>
<pre>
  replaceable package MediumA = Buildings.Media.Air \"Medium for air\";
</pre>
<p>
This will allow the propagation of the medium model to all models that contain air.
In this example, there is only one model with air, but when we connect an air
supply, there will be multiple models that use this medium.
</p>
<p>
We called the medium <code>MediumA</code> to distinguish it from
<code>MediumW</code> that we will use in later versions of the model for components that
have water as a medium.
</p>
<p>
Note that although the medium model is for unsaturated air, the cooling coil
will be able to reduce the moisture content of the medium. Because
the air outlet state of the cooling coil has a relative humidity below <i>100%</i>,
we can use this medium model and need not be able to model the fog region.
</p>
<p>
We also defined the system-level parameters
</p>
<pre>
  parameter Modelica.Units.SI.Volume V=6*10*3 \"Room volume\";
  parameter Modelica.Units.SI.MassFlowRate mA_flow_nominal = V*1.2*6/3600
    \"Nominal mass flow rate\";
  parameter Modelica.Units.SI.HeatFlowRate QRooInt_flow = 1000
    \"Internal heat gains of the room\";
</pre>
<p>
to declare that the room volume is <i>180</i> m<sup>3</sup>, that the room
has a nominal mass flow rate of <i>6</i> air changes per hour and that the internal heat gains of the room are <i>1000</i> Watts.
These parameters have been declared at the top-level of the model
as they will be used in several other models.
Declaring them at the top-level allows to propagate them to other
models, and to easily change them at one location should this be required
when revising the model.
</p>
</li>
<li>
<p>
To model the room air, approximated as a completely mixed volume of air,
an instance of
<a href=\"modelica://Buildings.Fluid.MixingVolumes.MixingVolume\">
Buildings.Fluid.MixingVolumes.MixingVolume</a>
has been used, as this model can be used with dry air or moist air.
The medium model has been set to <code>MediumA</code>.
We set the parameter
<code>energyDynamics=Modelica.Fluid.Types.Dynamics.FixedInitial</code>
which will cause the initial conditions of the volume to be fixed to
the values specified by the parameters on the <code>Initialization</code>
tab.
</p>
<p>
The nominal mass flow rate of the volume is set to <code>mA_flow_nominal</code>.
The nominal mass flow rate is used for numerical reasons and should be set
to the approximate order of magnitude. It only has an effect if the mass flow
rate is near zero and what \"near zero\" means depends on the magnitude of
<code>m_flow_nominal</code>, as it is used for the default value of the parameter
<code>m_flow_small</code> on the <code>Assumptions</code> tag of the model.
See also
<a href=\"modelica://Buildings.Fluid.UsersGuide\">
Buildings.Fluid.UsersGuide</a>
for an explanation of the purpose of <code>m_flow_small</code>.
</p>
</li>
<li>
<p>
To increase the heat capacity of the room air to approximate
energy storage in furniture and building constructions, we set the parameter
<code>mSenFac=3</code> in the instance <code>vol</code>.
This will increase the sensible heat capacity
of the room air by a factor of three.
</p>
</li>
<li>
<p>
We used the instance <code>heaCon</code> to model the heat conductance to the ambient.
Since our room should have a heat loss of <i>10</i> kW at a temperature difference
of <i>30</i> Kelvin, we set the conductance to
<i>G=10000 &frasl; 30</i> W/K.
</p>
</li>
<li>
<p>
Finally, we used the instance <code>preHea</code> to model a prescribed, constant heat gain of
<i>1000</i> Watts, such as due to internal heat source.
</p>
</li>
</ol>
<p>
This completes the initial version of the model. When simulating the model
for <i>3</i> hours, or <i>10800</i> seconds, the
response shown below should be seen.
</p>
<p align=\"center\">
<img alt=\"image\" src=\"modelica://Buildings/Resources/Images/Examples/Tutorial/SpaceCooling/System1Temperatures.png\" border=\"1\"/>
</p>
<p>
To verify the correctness of the model, we can compare the simulated results to the
following analytical solutions:
</p>
<ol>
<li>
At steady-state, the temperature difference to the outside should be
<i>&Delta; T = Q&#775; &frasl; UA = 1000/(10000/30) = 3</i> Kelvin, which
corresponds to a room temperature of <i>-7</i>&deg;C.
</li>
<li>
It can be shown that the time constant of the room is
<i>&tau; = C &frasl; UA = 1950</i> seconds, where
<i>C</i> is the heat capacity of the room air and the thermal storage element
that is connected to it, and
<i>G</i> is the heat conductance.
</li>
</ol>
<p>
Both analytical values agree with the simulation results shown in the above figure.
</p>
<!-- Notes -->
<h4>Notes</h4>
<p>
For a more realistic model of a room, the model
<a href=\"modelica://Buildings.ThermalZones.Detailed.MixedAir\">
Buildings.ThermalZones.Detailed.MixedAir</a>
could have been used.
For transient heat conduction, models from the
package
<a href=\"modelica://Buildings.HeatTransfer.Conduction\">
Buildings.HeatTransfer.Conduction</a>
could have been used.
</p>
</html>", revisions="<html>
<ul>
<li>
April 21, 2021, by Michael Wetter:<br/>
Corrected error in calculation of design mass flow rate.<br/>
This is for
<a href=\"https://github.com/lbl-srg/modelica-buildings/issues/2458\">#2458</a>.
</li>
<li>
January 28, 2015 by Michael Wetter:<br/>
Added thermal mass of furniture directly to air volume.
This avoids an index reduction.
</li>
<li>
December 22, 2014 by Michael Wetter:<br/>
Removed <code>Modelica.Fluid.System</code>
to address issue
<a href=\"https://github.com/lbl-srg/modelica-buildings/issues/311\">#311</a>.
</li>
<li>
January 11, 2012, by Michael Wetter:<br/>
First implementation.
</li>
</ul>
</html>"),
    __Dymola_Commands(file=
     "modelica://Buildings/Resources/Scripts/Dymola/Examples/Tutorial/SpaceCooling/System1.mos"
        "Simulate and plot"),
    experiment(
      StartTime=15552000,
      StopTime=15638400,
      Tolerance=1e-06,
      __Dymola_Algorithm="Dassl"),
    Diagram(coordinateSystem(extent={{-180,-100},{140,120}})),
    Icon(coordinateSystem(extent={{-180,-100},{140,120}})));
end System1_SC;
