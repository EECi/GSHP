within Buildings.Utilities.IO.Python_3_8.Functions.Examples;
model Exchange "Test model for exchange function"
  extends Modelica.Icons.Example;

  parameter Boolean passPythonObject = false
    "Set to true if the Python function returns and receives an object, see User's Guide";

  parameter Real    yR1[1](each fixed=false) "Real function value";
  parameter Integer yI1[1](each fixed=false) "Integer function value";
  parameter Real    yR2[2](each fixed=false) "Real function value";
  parameter Integer yI2[2](each fixed=false) "Integer function value";

protected
  model M "Class that contains the Python object"
    Buildings.Utilities.IO.Python_3_8.Functions.BaseClasses.PythonObject pytObj=
        Buildings.Utilities.IO.Python_3_8.Functions.BaseClasses.PythonObject()
      "Instance of Python object";
  end M;

  M[8] m "Array with instances of Python objects";

initial algorithm
  yR1 := Buildings.Utilities.IO.Python_3_8.Functions.exchange(
    moduleName="testFunctions",
    functionName="r1_r1",
    pytObj=m[1].pytObj,
    passPythonObject=passPythonObject,
    dblWri={2.0},
    intWri={0},
    nDblWri=1,
    nDblRea=1,
    nIntWri=0,
    nIntRea=0,
    nStrWri=0,
    strWri={""});
    assert(abs(4-yR1[1]) < 1e-5, "Error in function r1_r1");

  yR1 := Buildings.Utilities.IO.Python_3_8.Functions.exchange(
    moduleName="testFunctions",
    functionName="r2_r1",
    pytObj=m[2].pytObj,
    passPythonObject=passPythonObject,
    dblWri={2.0,3.0},
    intWri={0},
    nDblWri=2,
    nDblRea=1,
    nIntWri=0,
    nIntRea=0,
    nStrWri=0,
    strWri={""});
    assert(abs(6-yR1[1]) < 1e-5, "Error in function r2_r1");

  yR2 := Buildings.Utilities.IO.Python_3_8.Functions.exchange(
    moduleName="testFunctions",
    functionName="r1_r2",
    pytObj=m[3].pytObj,
    passPythonObject=passPythonObject,
    dblWri={2.0},
    intWri={0},
    nDblWri=1,
    nDblRea=2,
    nIntWri=0,
    nIntRea=0,
    nStrWri=0,
    strWri={""});
  assert(abs(yR2[1]-2) + abs(yR2[2]-4) < 1E-5, "Error in function r1_r2");

  // In the call below, yR1 is a dummy variable, as exchange returns (Real[1], Integer[1])
  (yR1,yI1) := Buildings.Utilities.IO.Python_3_8.Functions.exchange(
    moduleName="testFunctions",
    functionName="i1_i1",
    pytObj=m[4].pytObj,
    passPythonObject=passPythonObject,
    dblWri={0.0},
    intWri={3},
    nDblWri=0,
    nDblRea=0,
    nIntWri=1,
    nIntRea=1,
    nStrWri=0,
    strWri={""});
  assert((6-yI1[1]) < 1e-5, "Error in function i1_i1");

  // In the call below, yR1 is a dummy variable, as exchange returns (Real[1], Integer[2])
  (yR1,yI2) := Buildings.Utilities.IO.Python_3_8.Functions.exchange(
    moduleName="testFunctions",
    functionName="i1_i2",
    pytObj=m[5].pytObj,
    passPythonObject=passPythonObject,
    dblWri={0.0},
    intWri={2},
    nDblWri=0,
    nDblRea=0,
    nIntWri=1,
    nIntRea=2,
    nStrWri=0,
    strWri={""});
  assert(abs(yI2[1]-2) + abs(yI2[2]-4) < 1E-5, "Error in function i1_i2");

  yR2 := Buildings.Utilities.IO.Python_3_8.Functions.exchange(
    moduleName="testFunctions",
    functionName="r1i1_r2",
    pytObj=m[6].pytObj,
    passPythonObject=passPythonObject,
    dblWri={0.3},
    intWri={2},
    nDblWri=1,
    nDblRea=2,
    nIntWri=1,
    nIntRea=0,
    nStrWri=0,
    strWri={""});
  assert(abs(yR2[1]-0.6) + abs(yI2[2]-4) < 1E-5, "Error in function r1i1_r2");

  // From Modelica, write a number to a text file, and from Python, read the text file
  // and return the number.
  Modelica.Utilities.Streams.print(string="1.23", fileName="tmp-TestPythonInterface.txt");
  yR1 := Buildings.Utilities.IO.Python_3_8.Functions.exchange(
    moduleName="testFunctions",
    functionName="s2_r1",
    pytObj=m[7].pytObj,
    passPythonObject=passPythonObject,
    dblWri={0.0},
    intWri={0},
    nDblWri=0,
    nDblRea=1,
    nIntWri=0,
    nIntRea=0,
    nStrWri=2,
    strWri={"tmp-TestPythonInterface","txt"});
   assert(abs(yR1[1]-1.23) < 1E-5, "Error in function s2_r1");

  (yR2,yI1) := Buildings.Utilities.IO.Python_3_8.Functions.exchange(
    moduleName="testFunctions",
    functionName="r1i1_r2i1",
    pytObj=m[8].pytObj,
    passPythonObject=passPythonObject,
    dblWri={0.3},
    intWri={2},
    nDblWri=1,
    nDblRea=2,
    nIntWri=1,
    nIntRea=1,
    nStrWri=0,
    strWri={""});
  assert(abs(yR2[1]-0.6) + abs(yR2[2]-4) < 1E-5, "Error in function r1i1_r2i1");
  assert(abs(yI1[1]-3) == 0, "Error in function r1i1_r2i1");
  annotation (
experiment(Tolerance=1e-6, StopTime=1.0),
__Dymola_Commands(file="modelica://Buildings/Resources/Scripts/Dymola/Utilities/IO/Python_3_8/Functions/Examples/Exchange.mos"
        "Simulate and plot"),
Documentation(info="<html>
<p>
This example calls various functions in the Python module <code>testFunctions.py</code>.
It tests whether arguments and return values are passed correctly.
The functions in  <code>testFunctions.py</code> are very simple in order to test
whether they compute correctly, and whether the data conversion between Modelica and
Python is implemented correctly.
Each call to Python is followed by an <code>assert</code> statement which terminates
the simulation if the return value is different from the expected value.
</p>
</html>", revisions="<html>
<ul>
<li>
January 10, 2022, by Michael Wetter:<br/>
Updated to Python 3.8.
</li>
<li>
December 11, 2021, by Michael Wetter:<br/>
Changed implementation to assigning parameters as opposed to variables because the
assignment is made through impure function calls.
This is for MSL 4.0.0.
</li>
<li>
December 10, 2021, by Michael Wetter:<br/>
Removed call to impure function <code>removeFile</code>.
This removal is required for MSL 4.0.0.
</li>
<li>
April 10, 2020, by Jianjun Hu and Michael Wetter:<br/>
Updated to Python 3.6.
</li>
<li>
February 2, 2016, by Michael Wetter:<br/>
Changed constant arguments of exchange function from <code>int</code> to <code>double</code>.
This is required for OpenModelica.
</li>
<li>
January 31, 2013, by Michael Wetter:<br/>
First implementation.
</li>
</ul>
</html>"));
end Exchange;
