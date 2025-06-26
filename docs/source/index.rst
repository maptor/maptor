MAPTOR: Multiphase Adaptive Trajectory Optimizer
================================================

**Date**: |today| **Version**: |version|

**Useful links**:
`Install <https://pypi.org/project/maptor/>`__ |
`Source Repository <https://github.com/maptor/maptor>`__ |
`Issues & Ideas <https://github.com/maptor/maptor/issues>`__ |

**MAPTOR** is a Python framework for **trajectory and design optimization** using optimal control. MAPTOR simultaneously optimizes system parameters and trajectories for vehicles, robots, spacecraft, and other dynamic systems.

.. grid:: 1 1 2 2
    :gutter: 2 3 4 4

    .. grid-item-card::
        :text-align: center

        **Getting Started**
        ^^^

        Get up and running with MAPTOR in 5 minutes.
        Learn the basic problem definition pattern and solve your first optimal control problem.

        +++

        .. button-ref:: quickstart
            :color: primary
            :click-parent:

            Get Started

    .. grid-item-card::
        :text-align: center

        **Tutorials**
        ^^^

        Comprehensive guides covering complete problem definition including design parameters,
        solution data access, and best practices for multiphase optimal control.

        +++

        .. button-ref:: tutorials/index
            :color: primary
            :click-parent:

            Learn More

    .. grid-item-card::
        :text-align: center

        **Examples Gallery**
        ^^^

        Complete, runnable examples from aerospace to robotics.
        Each example includes mathematical formulation and detailed implementation.

        +++

        .. button-ref:: examples/index
            :color: primary
            :click-parent:

            Browse Examples

    .. grid-item-card::
        :text-align: center

        **API Reference**
        ^^^

        Comprehensive reference documentation for all public classes,
        functions, and methods in the MAPTOR framework.

        +++

        .. button-ref:: api/index
            :color: primary
            :click-parent:

            API Docs


When to Use MAPTOR
------------------

**If you only need basic PATH planning** (geometry-focused problems):

* A*, Dijkstra, basic RRT, PRM
* Fastest for obstacle avoidance without complex dynamics

**If you need TRAJECTORY optimization** with simple constraints:

* iLQR (iterative Linear Quadratic Regulator)
* ALTRO (Augmented Lagrangian Trajectory Optimizer)
* Faster convergence for dynamics-heavy, constraint-light problems

**Use MAPTOR for complex DESIGN + TRAJECTORY problems:**

* Multiple design parameters + trajectory optimization
* Complex nonlinear path constraints (obstacle avoidance, state bounds)
* Multiphase missions with automatic phase linking
* When you need the full flexibility of direct transcription


Installation
------------

.. code-block:: bash

    pip install maptor

Documentation
-------------

.. toctree::
   :maxdepth: 2

   installation
   quickstart
   tutorials/index
   examples/index
   api/index

Indices and Tables
==================

* :ref:`genindex`
* :ref:`modindex`
* :ref:`search`
