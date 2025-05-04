# To generate Go code from proto
#### cd to the root dir then
``protoc --proto_path=proto \
  --go_out=GO/pb --go_opt=paths=source_relative \
  --go-grpc_out=GO/pb --go-grpc_opt=paths=source_relative \
  proto/<name>.proto
``
#### Then go to /GO/pb and create a directory holding <name> and move the generated `` <name>.pb.go`` `` <name>.pb_grpc.go`` in it
# To generate Python code from proto
#### cd to root dir then
``
python3 -m grpc_tools.protoc \
  -Iproto \
  --python_out=ros/src/control/data_contracts \
  --grpc_python_out=ros/src/control/data_contracts \
  proto/<name>.proto
``
#### Then go to the generated `` <name>_pb2_grpc.py`` change the  `` import <name>_pb2 as <name>__pb2`` to `` from . import <name>_pb2 as <name>__pb2``