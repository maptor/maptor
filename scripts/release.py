"""
Usage:
    python scripts/release.py check           # Check current status
    python scripts/release.py bump patch      # 0.0.1 -> 0.0.2
    python scripts/release.py bump minor      # 0.0.1 -> 0.1.0
    python scripts/release.py bump major      # 0.0.1 -> 1.0.0
    python scripts/release.py release 0.1.0   # Full release process
"""

import subprocess
import sys
import tomllib
import re
from pathlib import Path
from typing import Tuple


class ReleaseManager:
    def __init__(self):
        self.project_root = Path(__file__).parent.parent
        self.pyproject_path = self.project_root / "pyproject.toml"
        self.init_path = self.project_root / "maptor" / "__init__.py"

    def get_current_version(self) -> str:
        """Get current version from pyproject.toml"""
        with open(self.pyproject_path, "rb") as f:
            data = tomllib.load(f)
        return data["project"]["version"]

    def bump_version(self, bump_type: str) -> str:
        """Bump version according to semantic versioning"""
        current = self.get_current_version()
        major, minor, patch = map(int, current.split("."))
        
        if bump_type == "major":
            major += 1
            minor = 0
            patch = 0
        elif bump_type == "minor":
            minor += 1
            patch = 0
        elif bump_type == "patch":
            patch += 1
        else:
            raise ValueError(f"Invalid bump type: {bump_type}")
        
        return f"{major}.{minor}.{patch}"

    def update_version_files(self, new_version: str) -> None:
        """Update version in both pyproject.toml and __init__.py"""
        # Update pyproject.toml
        with open(self.pyproject_path, "r") as f:
            content = f.read()
        
        content = re.sub(
            r'version = "[^"]*"',
            f'version = "{new_version}"',
            content
        )
        
        with open(self.pyproject_path, "w") as f:
            f.write(content)
        
        # Update __init__.py
        with open(self.init_path, "r") as f:
            init_content = f.read()
        
        init_content = re.sub(
            r'__version__ = "[^"]*"',
            f'__version__ = "{new_version}"',
            init_content
        )
        
        with open(self.init_path, "w") as f:
            f.write(init_content)
        
        print(f"Updated version to {new_version} in:")
        print(f"   • {self.pyproject_path}")
        print(f"   • {self.init_path}")

    def check_git_status(self) -> Tuple[bool, str]:
        """Check if git repo is clean"""
        try:
            result = subprocess.run(
                ["git", "status", "--porcelain"],
                capture_output=True,
                text=True,
                check=True
            )
            is_clean = len(result.stdout.strip()) == 0
            return is_clean, result.stdout
        except subprocess.CalledProcessError as e:
            return False, f"Git error: {e}"

    def create_release_commit_and_tag(self, version: str) -> None:
        """Create release commit and tag"""
        # Commit version changes
        subprocess.run(["git", "add", str(self.pyproject_path), str(self.init_path)], check=True)
        subprocess.run(["git", "commit", "-m", f"Release v{version}"], check=True)
        
        # Create tag
        subprocess.run(["git", "tag", f"v{version}"], check=True)
        
        print(f"Created release commit and tag v{version}")

    def check_status(self) -> None:
        """Show current release status"""
        current_version = self.get_current_version()
        is_clean, git_output = self.check_git_status()
        
        print("MAPTOR Release Status:")
        print(f"   Current version: {current_version}")
        print(f"   Git status: {'Clean' if is_clean else 'Uncommitted changes'}")
        
        if not is_clean:
            print("   Uncommitted files:")
            for line in git_output.strip().split('\n'):
                if line:
                    print(f"     {line}")
        
        # Check if this version exists on PyPI
        try:
            import requests
            url = f"https://pypi.org/pypi/maptor/{current_version}/json"
            response = requests.get(url)
            if response.status_code == 200:
                print(f"   PyPI status: Version {current_version} EXISTS on PyPI")
            else:
                print(f"   PyPI status: Version {current_version} NOT on PyPI (ready to publish)")
        except ImportError:
            print("   PyPI status: Install 'requests' to check PyPI status")

    def perform_release(self, version: str) -> None:
        """Perform complete release process"""
        print(f"Starting release process for v{version}")
        
        # Check git status
        is_clean, _ = self.check_git_status()
        if not is_clean:
            print("Git repository has uncommitted changes. Commit them first.")
            return
        
        # Update version files
        self.update_version_files(version)
        
        # Create commit and tag
        self.create_release_commit_and_tag(version)
        
        print(f"""
Release v{version} prepared successfully!

Next steps:
1. Push to GitHub (this will trigger PyPI release):
   git push origin main --tags

2. Monitor the release workflow:
   https://github.com/YOUR_USERNAME/maptor/actions

3. Verify on PyPI (after workflow completes):
   https://pypi.org/project/maptor/{version}/

4. Test installation:
   pip install maptor=={version}
""")


def main():
    if len(sys.argv) < 2:
        print(__doc__)
        return
    
    manager = ReleaseManager()
    command = sys.argv[1]
    
    try:
        if command == "check":
            manager.check_status()
        
        elif command == "bump":
            if len(sys.argv) != 3:
                print("Usage: python scripts/release.py bump [major|minor|patch]")
                return
            
            bump_type = sys.argv[2]
            if bump_type not in ["major", "minor", "patch"]:
                print("Bump type must be: major, minor, or patch")
                return
            
            current = manager.get_current_version()
            new_version = manager.bump_version(bump_type)
            print(f"Would bump version: {current} -> {new_version}")
            
            confirm = input("Proceed? (y/N): ")
            if confirm.lower() == 'y':
                manager.update_version_files(new_version)
                print(f"Version bumped to {new_version}")
                print("   Use 'git commit' to commit the changes")
            else:
                print("Version bump cancelled")
        
        elif command == "release":
            if len(sys.argv) != 3:
                print("Usage: python scripts/release.py release VERSION")
                return
            
            version = sys.argv[2]
            if not re.match(r'^\d+\.\d+\.\d+$', version):
                print("Version must be in format X.Y.Z (e.g., 0.1.0)")
                return
            
            current = manager.get_current_version()
            print(f"Creating release: {current} -> {version}")
            
            confirm = input("This will create a git commit and tag. Proceed? (y/N): ")
            if confirm.lower() == 'y':
                manager.perform_release(version)
            else:
                print("Release cancelled")
        
        else:
            print(f"Unknown command: {command}")
            print(__doc__)
    
    except Exception as e:
        print(f"Error: {e}")
        sys.exit(1)


if __name__ == "__main__":
    main()