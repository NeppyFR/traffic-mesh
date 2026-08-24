package trafficmesh;

import java.util.ArrayList;
import java.util.LinkedHashMap;
import java.util.List;
import java.util.Map;

/**
 * A minimal, dependency-free JSON reader — just enough to hand raw Overpass
 * output to {@link Osm#buildGraphFromOverpass}. The JS port gets this for free
 * from {@code JSON.parse}; Java does not, and the project's zero-dependency
 * ethos rules out a library.
 *
 * <p>Values map to: {@link LinkedHashMap} (object, insertion-ordered like a JS
 * object), {@link ArrayList} (array), {@link String}, {@link Long} for integral
 * numbers, {@link Double} otherwise, {@link Boolean}, and {@code null}.
 */
public final class Json {

  private final String src;
  private int i;

  private Json(String src) {
    this.src = src;
    this.i = 0;
  }

  public static Object parse(String text) {
    Json p = new Json(text);
    p.ws();
    Object v = p.value();
    p.ws();
    if (p.i < p.src.length()) throw p.err("trailing content");
    return v;
  }

  /** Convenience for the common case of a top-level object. */
  @SuppressWarnings("unchecked")
  public static Map<String, Object> parseObject(String text) {
    Object v = parse(text);
    if (!(v instanceof Map)) throw new IllegalArgumentException("expected a JSON object");
    return (Map<String, Object>) v;
  }

  private RuntimeException err(String msg) {
    return new IllegalArgumentException("JSON at offset " + i + ": " + msg);
  }

  private void ws() {
    while (i < src.length()) {
      char c = src.charAt(i);
      if (c == ' ' || c == '\t' || c == '\n' || c == '\r') i++;
      else break;
    }
  }

  private char peek() {
    if (i >= src.length()) throw err("unexpected end of input");
    return src.charAt(i);
  }

  private void expect(char c) {
    if (peek() != c) throw err("expected '" + c + "' but found '" + peek() + "'");
    i++;
  }

  private Object value() {
    char c = peek();
    switch (c) {
      case '{':
        return object();
      case '[':
        return array();
      case '"':
        return string();
      case 't':
        literal("true");
        return Boolean.TRUE;
      case 'f':
        literal("false");
        return Boolean.FALSE;
      case 'n':
        literal("null");
        return null;
      default:
        return number();
    }
  }

  private void literal(String word) {
    if (!src.startsWith(word, i)) throw err("expected " + word);
    i += word.length();
  }

  private Map<String, Object> object() {
    expect('{');
    LinkedHashMap<String, Object> out = new LinkedHashMap<>();
    ws();
    if (peek() == '}') {
      i++;
      return out;
    }
    while (true) {
      ws();
      String key = string();
      ws();
      expect(':');
      ws();
      out.put(key, value());
      ws();
      char c = peek();
      if (c == ',') {
        i++;
        continue;
      }
      if (c == '}') {
        i++;
        return out;
      }
      throw err("expected ',' or '}'");
    }
  }

  private List<Object> array() {
    expect('[');
    List<Object> out = new ArrayList<>();
    ws();
    if (peek() == ']') {
      i++;
      return out;
    }
    while (true) {
      ws();
      out.add(value());
      ws();
      char c = peek();
      if (c == ',') {
        i++;
        continue;
      }
      if (c == ']') {
        i++;
        return out;
      }
      throw err("expected ',' or ']'");
    }
  }

  private String string() {
    expect('"');
    StringBuilder sb = new StringBuilder();
    while (true) {
      if (i >= src.length()) throw err("unterminated string");
      char c = src.charAt(i++);
      if (c == '"') return sb.toString();
      if (c != '\\') {
        sb.append(c);
        continue;
      }
      char e = src.charAt(i++);
      switch (e) {
        case '"' -> sb.append('"');
        case '\\' -> sb.append('\\');
        case '/' -> sb.append('/');
        case 'b' -> sb.append('\b');
        case 'f' -> sb.append('\f');
        case 'n' -> sb.append('\n');
        case 'r' -> sb.append('\r');
        case 't' -> sb.append('\t');
        case 'u' -> {
          sb.append((char) Integer.parseInt(src.substring(i, i + 4), 16));
          i += 4;
        }
        default -> throw err("bad escape \\" + e);
      }
    }
  }

  /** Integral literals become Long so String.valueOf() matches JS String(1) == "1". */
  private Object number() {
    int start = i;
    if (peek() == '-') i++;
    boolean fractional = false;
    while (i < src.length()) {
      char c = src.charAt(i);
      if (c >= '0' && c <= '9') {
        i++;
      } else if (c == '.' || c == 'e' || c == 'E' || c == '+' || c == '-') {
        fractional = true;
        i++;
      } else {
        break;
      }
    }
    String tok = src.substring(start, i);
    if (tok.isEmpty() || tok.equals("-")) throw err("invalid number");
    if (!fractional) {
      try {
        return Long.parseLong(tok);
      } catch (NumberFormatException ignored) {
        // fall through to double for values beyond long range
      }
    }
    return Double.parseDouble(tok);
  }
}
