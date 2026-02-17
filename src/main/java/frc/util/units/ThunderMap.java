package frc.util.units;

import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.Unit;

@SuppressWarnings("rawtypes")
public class ThunderMap<M1 extends Measure, M2 extends Measure> extends InterpolatingDoubleTreeMap {

    private Unit valueUnit;
    
    /**
     * Creates a new ThunderMap.
     * This is a class for linear interpolation using WPILib units
     * Use {@code put(M1 key, M2 value)} to add values to the map and {@code get(M1 key)} to retrieve values from the map. 
     * The map will automatically convert between different units of the same type (e.g. meters and feet) 
     * and perform linear interpolation between values.
     * 
     * See example usage in {@code UnitsTest.testThunderMap()}.
     */
    public ThunderMap() {
        super();
    }

    /**
     * Adds a key-value pair to the map. The key and value can be in any unit, but they must be of the types M1 and M2 respectively 
     * (e.g. Distance and Angle).
     * 
     * @param key
     * @param value
     */
    public void put(M1 key, M2 value) {
        valueUnit = value.baseUnit();
        super.put(key.baseUnitMagnitude(), value.baseUnitMagnitude());
    }

    /**
     * Retrieves a value from the map corresponding to the given key. The key can be in any unit, 
     * but it must be of the same type as M1.
     * @param key
     * @return The value corresponding to the given key, converted to type M2 (e.g. Angle)
     */
    @SuppressWarnings("unchecked")
    public M2 get(M1 key) {
        return (M2) valueUnit.of(super.get(key.baseUnitMagnitude()));
    }

    @Deprecated()
    @Override
    public void put(Double key, Double value) {
        throw new UnsupportedOperationException("Use put(M1 key, M2 value) instead");
    }

    @Deprecated
    @Override
    public Double get(Double key) {
        throw new UnsupportedOperationException("Use get(M1 key) instead");
    }
}