import sys
from pathlib import Path


def generate_api_docs():
    """
    Generate API documentation for PUBLIC interfaces only.

    Only documents what's in maptor.__all__ - the official public API.
    """
    # Determine project root (docs/source/ -> project_root)
    project_root = Path(__file__).resolve().parent.parent.parent
    sys.path.insert(0, str(project_root))

    print(f"Project root: {project_root}")
    print("Generating PUBLIC API documentation only...")

    try:
        import maptor

        print(f"MAPTOR imported from: {maptor.__file__}")
    except ImportError as e:
        print(f"Error importing MAPTOR: {e}")
        print("Ensure MAPTOR is installed: pip install -e .")
        return

    # Create output directory
    api_dir = Path(__file__).parent / "api"
    api_dir.mkdir(exist_ok=True)
    print(f"Output directory: {api_dir}")

    # Define ONLY the public API - matches maptor.__all__
    PUBLIC_API_MODULES = {
        "maptor": {
            "title": "Main Package",
            "description": "The primary MAPTOR interface with all public exports.",
            "modules": ["maptor"],
        },
        "problem": {
            "title": "Problem Definition",
            "description": "Classes for defining optimal control problems.",
            "modules": ["maptor.problem.core_problem"],
        },
        "solution": {
            "title": "Solution Interface",
            "description": "Classes for working with optimization results.",
            "modules": ["maptor.solution"],
        },
        "solver": {
            "title": "Solver Functions",
            "description": "Functions for solving optimal control problems.",
            "modules": ["maptor.solver"],
        },
        "exceptions": {
            "title": "Exceptions",
            "description": "Exception classes for error handling.",
            "modules": ["maptor.exceptions"],
        },
    }

    # Generate RST files for each public module
    all_generated_modules = []

    for _category_key, category_info in PUBLIC_API_MODULES.items():
        print(f"  {category_info['title']}")

        for module_name in category_info["modules"]:
            # Create safe filename
            safe_name = module_name.replace(".", "_")
            rst_path = api_dir / f"{safe_name}.rst"

            # Generate RST content
            rst_content = f"""
{module_name}
{"=" * len(module_name)}

{category_info["description"]}

.. automodule:: {module_name}
   :members:
   :undoc-members:
   :show-inheritance:
   :exclude-members: __dict__,__weakref__
"""

            # Write RST file
            rst_path.write_text(rst_content.strip() + "\n")
            all_generated_modules.append((safe_name, module_name, category_info["title"]))
            print(f"    {rst_path.name}")

    # Generate the main API index
    index_content = f"""
API Reference
=============

This is the complete **public API** for MAPTOR.

.. important::

   If a class, function, or module is not listed here, it's an internal
   implementation detail and should not be used directly in your code.

The public API consists of only {len(all_generated_modules)} modules that provide everything
you need to define and solve optimal control problems.

Quick Start
-----------

.. code-block:: python

   import maptor as mtor

   # Define problem
   problem = mtor.Problem("My Problem")

   # Define variables and dynamics
   t = problem.time()
   x = problem.state("position")
   u = problem.control("thrust")
   problem.dynamics({{x: u}})
   problem.minimize(u**2)

   # Set mesh and solve
   problem.mesh([5], [-1, 1])
   solution = mtor.solve_fixed_mesh(problem)
   solution.plot()

"""

    # Add categorized module documentation
    for _category_key, category_info in PUBLIC_API_MODULES.items():
        title = category_info["title"]
        description = category_info["description"]

        index_content += f"\n{title}\n{'-' * len(title)}\n\n"
        index_content += f"{description}\n\n"
        index_content += ".. toctree::\n   :maxdepth: 1\n\n"

        for module_name in category_info["modules"]:
            safe_name = module_name.replace(".", "_")
            index_content += f"   {safe_name}\n"

    # Write main index
    index_path = api_dir / "index.rst"
    index_path.write_text(index_content)

    print(f"\nGenerated API index: {index_path.name}")
    print("PUBLIC API documentation complete!")
    print(f"Generated {len(all_generated_modules)} module docs in {api_dir}")
    print(
        f"Total public modules: {sum(len(cat['modules']) for cat in PUBLIC_API_MODULES.values())}"
    )


if __name__ == "__main__":
    generate_api_docs()
