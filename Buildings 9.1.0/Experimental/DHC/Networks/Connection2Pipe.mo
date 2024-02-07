within Buildings.Experimental.DHC.Networks;
model Connection2Pipe
  "Model for connecting an agent to a two-pipe distribution network, using fixed resistance pipe model"
  extends Buildings.Experimental.DHC.Networks.BaseClasses.PartialConnection2Pipe(
    redeclare model Model_pipDis=Fluid.FixedResistances.PressureDrop (
      final dp_nominal=dpDis_nominal),
    redeclare model Model_pipCon=Fluid.FixedResistances.LosslessPipe);
  parameter Modelica.Units.SI.PressureDifference dpDis_nominal
    "Pressure drop in distribution line (supply only, not counting return line)";
  annotation (
    Documentation(
      info="<html>
<p>
This is a model of a connection with a two-pipe distribution network using 
as pipe model a fixed hydraulic resistance with no heat loss .
</p>
</html>",
      revisions="<html>
<ul>
<li>
February 21, 2020, by Antoine Gautier:<br/>
First implementation.
</li>
</ul>
</html>"));
end Connection2Pipe;