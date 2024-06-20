import pkgutil

# Replace 'your_package' with the package name you want to inspect
package_name = 'pf_trunk_width'
package = __import__(package_name)

# List all modules in the specified package
modules = [name for _, name, _ in pkgutil.walk_packages(package.__path__, package.__name__ + '.')]

print(modules)