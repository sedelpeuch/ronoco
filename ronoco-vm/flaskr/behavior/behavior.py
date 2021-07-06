import py_trees
from flaskr import behavior
from flaskr import cartesian_point

types = {'selector': py_trees.composites.Selector,
         'sequence': py_trees.composites.Sequence,
         'parallel': py_trees.composites.Parallel,
         'execute': behavior.execute.Execute,
         'plan': behavior.plan.Plan,
         'place': behavior.plan.Plan}

composites = {'selector', 'sequence', 'parallel'}
leaf = {'execute', 'plan'}

commander = cartesian_point.CartesianPoint().commander
