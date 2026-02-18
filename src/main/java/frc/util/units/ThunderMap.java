package frc.util.units;

import java.util.Map;

import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.Unit;

public class ThunderMap<K extends Measure<?>, V extends Measure<?>> extends InterpolatingDoubleTreeMap {

    private Unit valueUnit;
    
    /**
     * Creates a new ThunderMap.
     * This is a class for linear interpolation using WPILib units
     * Use {@code put(K key, V value)} to add values to the map and {@code get(K key)} to retrieve values from the map. 
     * The map will automatically convert between different units of the same type (e.g. meters and feet) 
     * and perform linear interpolation between values.
     * 
     * See example usage in {@code UnitsTest.testThunderMap()}.
     */
    public ThunderMap() {}

    /**
     * Adds a key-value pair to the map. The key and value can be in any unit, but they must be of the types K and V respectively 
     * (e.g. Distance and Angle).
     * 
     * @param key
     * @param value
     */
    public void put(K key, V value) {
        valueUnit = value.baseUnit();
        super.put(key.baseUnitMagnitude(), value.baseUnitMagnitude());
    }

    /**
     * Retrieves a value from the map corresponding to the given key. The key can be in any unit, 
     * but it must be of the same type as K.
     * @param key
     * @return The value corresponding to the given key, converted to type V (e.g. Angle)
     */
    @SuppressWarnings("unchecked")
    public V get(K key) {
        return (V) valueUnit.of(super.get(key.baseUnitMagnitude()));
    }

    /**
     * Creates a {@link ThunderMap} from the given entries.
     *
     * @param <K> The type of the keys in the map, which must extend {@code Measure<?>}.
     * @param <V> The type of the values in the map, which must extend {@code Measure<?>}.
     * @param entries The entries to add to the map.
     * @return The map filled with the {@code entries}.
     */
    @SafeVarargs
    public static <K extends Measure<?>, V extends Measure<?>> ThunderMap<K, V> of(Map.Entry<K, V>... entries) {
        
        ThunderMap<K, V> map = new ThunderMap<>();

        for (var entry : entries) {
            map.put(entry.getKey(), entry.getValue());
        }

        return map;
    }

    /**
      * @deprecated Use {@code put(K key, V value)} and {@code get(K key)} instead. This method will throw an exception if used.
      */
    @Deprecated
    @Override
    public void put(Double key, Double value) {
        throw new UnsupportedOperationException("Use put(K key, V value) instead");
    }

    /**
      * @deprecated Use {@code put(K key, V value)} and {@code get(K key)} instead. This method will throw an exception if used.
      */
    @Deprecated
    @Override
    public Double get(Double key) {
        throw new UnsupportedOperationException("Use get(K key) instead");
    }

    /**
      * @deprecated Use {@code of(Map.Entry<K, V>... entries)} instead. This method will throw an exception if used.
      * 
      * @param entries The entries to add to the map.
      * @return The map filled with the {@code entries}.
      */
    @Deprecated
    @SafeVarargs
    public static InterpolatingDoubleTreeMap ofEntries(Map.Entry<Double, Double>... entries) {
        throw new UnsupportedOperationException("Use of(Map.Entry<K, V>... entries) instead");
    }
}