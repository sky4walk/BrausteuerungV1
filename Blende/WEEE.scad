
// Module names are of the form poly_<inkscape-path-id>().  As a result,
// you can associate a polygon in this OpenSCAD program with the corresponding
// SVG element in the Inkscape document by looking for the XML element with
// the attribute id="inkscape-path-id".

// fudge value is used to ensure that subtracted solids are a tad taller
// in the z dimension than the polygon being subtracted from.  This helps
// keep the resulting .stl file manifold.
fudge = 0.1;

module poly_path3337(h)
{
  scale([25.4/90, -25.4/90, 1]) union()
  {
    difference()
    {
       linear_extrude(height=h)
         polygon([[103.396500,47.615000],[103.874500,53.378000],[103.696060,56.911918],[103.172320,60.343717],[102.320655,63.656029],[101.158438,66.831484],[99.703040,69.852716],[97.971836,72.702354],[95.982198,75.363030],[93.751500,77.817375],[91.297114,80.048021],[88.636414,82.037600],[85.786772,83.768741],[82.765563,85.224078],[79.590157,86.386241],[76.277930,87.237861],[72.846253,87.761571],[69.312500,87.940000],[64.431291,87.598020],[59.759953,86.603094],[55.345045,85.001746],[51.233125,82.840500],[47.470752,80.165879],[44.104484,77.024406],[41.180881,73.462605],[38.746500,69.527000],[-45.773500,69.527000],[-45.773500,87.582000],[-68.010500,87.582000],[-68.010500,69.481000],[-69.909734,68.841953],[-71.565875,67.598125],[-72.775578,65.935984],[-73.335500,64.042000],[-78.083500,8.855000],[-195.884500,128.848000],[-202.926500,121.928000],[-79.188500,-3.990000],[-92.527500,-159.059000],[-210.243500,-283.234000],[-210.243500,-296.909000],[-93.840500,-174.317000],[-96.907500,-209.971000],[-111.363500,-209.971000],[-111.363500,-221.971000],[-98.644500,-221.971000],[-97.754500,-224.313000],[-95.961930,-227.849488],[-93.774063,-231.133719],[-91.218039,-234.174715],[-88.321000,-236.981500],[-85.110086,-239.563098],[-81.612438,-241.928531],[-73.865500,-246.047000],[-73.865500,-253.978000],[-53.360500,-253.978000],[-53.360500,-252.751000],[-45.255156,-254.389484],[-37.050250,-255.651375],[-20.861500,-257.219000],[-20.861500,-261.971000],[38.137500,-261.971000],[38.137500,-232.470000],[-20.861500,-232.470000],[-20.861500,-244.932000],[-37.088125,-243.222500],[-45.317453,-241.904062],[-53.360500,-240.223000],[-53.360500,-221.971000],[86.083500,-221.971000],[84.876163,-224.312408],[83.321664,-226.498111],[81.443271,-228.533325],[79.264250,-230.423266],[74.097398,-233.788193],[68.007250,-236.634625],[61.179945,-239.004291],[53.801625,-240.938922],[46.058430,-242.480248],[38.136500,-243.670000],[38.136500,-255.889000],[46.119994,-254.737861],[54.024516,-253.234141],[61.715498,-251.335756],[69.058375,-249.000625],[75.918580,-246.186666],[82.161547,-242.851797],[87.652709,-238.953936],[92.257500,-234.451000],[94.741766,-236.981203],[97.734625,-238.911875],[101.136422,-240.143359],[104.847500,-240.576000],[108.072186,-240.250955],[111.075609,-239.318703],[113.793447,-237.843568],[116.161375,-235.889875],[118.115068,-233.521947],[119.590203,-230.804109],[120.522455,-227.800686],[120.847500,-224.576000],[120.522455,-221.351602],[119.590203,-218.348312],[118.115068,-215.630492],[116.161375,-213.262500],[113.793447,-211.308695],[111.075609,-209.833438],[108.072186,-208.901086],[104.847500,-208.576000],[101.635500,-208.898500],[98.642500,-209.824000],[96.917500,-191.324000],[104.903500,-191.324000],[210.243500,-298.520000],[210.243500,-284.838000],[118.438500,-191.325000],[120.349500,-191.325000],[120.349500,-149.483000],[93.018500,-149.483000],[78.424500,7.105000],[186.379500,120.800000],[179.217500,127.597000],[103.396500,47.615000]]);
       translate([0, 0, -fudge])
         linear_extrude(height=h+2*fudge)
           polygon([[26.135500,-249.972000],[-8.863500,-249.972000],[-8.863500,-244.470000],[26.135500,-244.470000],[26.135500,-249.972000]]);
       translate([0, 0, -fudge])
         linear_extrude(height=h+2*fudge)
           polygon([[-73.866500,-232.891000],[-77.691016,-230.560250],[-80.999625,-227.972250],[-83.728672,-225.113875],[-85.814500,-221.972000],[-73.866500,-221.972000],[-73.866500,-232.891000]]);
       translate([0, 0, -fudge])
         linear_extrude(height=h+2*fudge)
           polygon([[107.844500,-180.533000],[94.659500,-167.103000],[94.022500,-160.273000],[109.847500,-160.273000],[109.847500,-180.533000],[107.844500,-180.533000]]);
       translate([0, 0, -fudge])
         linear_extrude(height=h+2*fudge)
           polygon([[-11.155500,-73.222000],[-79.281500,-145.086000],[-68.113500,-15.260000],[-11.155500,-73.222000]]);
       translate([0, 0, -fudge])
         linear_extrude(height=h+2*fudge)
           polygon([[2.319500,-73.045000],[67.448500,-4.453000],[80.964500,-149.483000],[79.631500,-149.483000],[79.631500,-151.797000],[2.319500,-73.045000]]);
       translate([0, 0, -fudge])
         linear_extrude(height=h+2*fudge)
           polygon([[-80.596500,-160.369000],[-4.388500,-80.109000],[40.688500,-125.979000],[-22.863500,-125.979000],[-22.863500,-149.979000],[50.637500,-149.979000],[50.637500,-136.104000],[79.631500,-165.608000],[79.631500,-191.323000],[84.863500,-191.323000],[86.554500,-209.474000],[-84.820500,-209.474000],[-80.596500,-160.369000]]);
       translate([0, 0, -fudge])
         linear_extrude(height=h+2*fudge)
           polygon([[34.748500,53.378000],[34.902407,50.094683],[35.354869,46.898242],[37.099891,40.821563],[39.872436,35.259102],[43.561375,30.322000],[48.055580,26.121398],[53.243922,22.768438],[59.015271,20.374258],[62.084846,19.571442],[65.258500,19.050000],[66.248500,8.429000],[-4.450500,-66.149000],[-67.010500,-2.425000],[-61.853500,57.528000],[34.994500,57.528000],[34.748500,53.378000]]);
       translate([0, 0, -fudge])
         linear_extrude(height=h+2*fudge)
           polygon([[51.336500,53.378000],[51.701660,57.000498],[52.748969,60.374547],[54.406168,63.427854],[56.601000,66.088125],[59.261207,68.283068],[62.314531,69.940391],[65.688715,70.987799],[69.311500,71.353000],[72.933998,70.987799],[76.308047,69.940391],[79.361354,68.283068],[82.021625,66.088125],[84.216568,63.427854],[85.873891,60.374547],[86.921299,57.000498],[87.286500,53.378000],[86.921299,49.755215],[85.873891,46.381031],[84.216568,43.327707],[82.021625,40.667500],[79.361354,38.472668],[76.308047,36.815469],[72.933998,35.768160],[69.311500,35.403000],[65.688715,35.768160],[62.314531,36.815469],[59.261207,38.472668],[56.601000,40.667500],[54.406168,43.327707],[52.748969,46.381031],[51.701660,49.755215],[51.336500,53.378000]]);
       translate([0, 0, -fudge])
         linear_extrude(height=h+2*fudge)
           polygon([[57.637500,53.378000],[57.874689,51.025389],[58.554953,48.834109],[59.631334,46.851119],[61.056875,45.123375],[62.784619,43.697834],[64.767609,42.621453],[66.958889,41.941189],[69.311500,41.704000],[71.664111,41.941189],[73.855391,42.621453],[75.838381,43.697834],[77.566125,45.123375],[78.991666,46.851119],[80.068047,48.834109],[80.748311,51.025389],[80.985500,53.378000],[80.748311,55.730568],[80.068047,57.921734],[78.991666,59.904564],[77.566125,61.632125],[75.838381,63.057482],[73.855391,64.133703],[71.664111,64.813854],[69.311500,65.051000],[66.958889,64.813854],[64.767609,64.133703],[62.784619,63.057482],[61.056875,61.632125],[59.631334,59.904564],[58.554953,57.921734],[57.874689,55.730568],[57.637500,53.378000]]);
    }
    linear_extrude(height=h)
      polygon([[-176.365500,213.859000],[171.307500,213.859000],[171.307500,298.520000],[-176.365500,298.520000],[-176.365500,213.859000]]);
  }
}

poly_path3337(5);
