branch derived from **knockout** with additional auto capabilities

## DEV NOTES
### pathweaver
- dont use auto routines, they are useless to us since commands cant be added through the gui
- ignore stuff online that says "paths need to be in deploy/paths" or something, pathweaver is dumb, ive implemented a hacky workaround involving `/home/lvuser/py`
    - also, ignore the `PathWeaver/Autos` and `PathWeaver/Paths` directories, only `PathWeaver/output` contains our juicy path jsons
- when importing the project in pathweaver, select the "PathWeaver" folder
- supposedly, reading path jsons can be slow? do it at auto init
- `PathWeaver/pathweaver.json` is our pathweaver config file, dont touch anything that isnt a number
    - paths will not build if numbers are set to 0.0