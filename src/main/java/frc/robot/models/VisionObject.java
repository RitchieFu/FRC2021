package frc.robot.models;

public class VisionObject {
    public String objectLabel; 
    public double x;
    public double y;
    public double z;
    public double confidence;
    

    public VisionObject(String objectType, double x, double y, double z)
    {
        this.objectLabel = objectType;
        this.x = x;
        this.y = y;
        this.z = z;
    }

};


/*
from marshmallow import Schema, fields


class VisionObject(Schema):
};


/*
from marshmallow import Schema, fields


class VisionObject(Schema):
    objectType = fields.Str(required=True)
    x = fields.Float(required=True)
    y = fields.Float(required=True)
    z = fields.Float(required=True)

class Something(Schema):
    items = fields.List(fields.Nested(VisionObject))
*/